import simd

/// Stage 2: `PilotInput` (normalized, craft-agnostic) → `DesiredCraftState` (physical targets),
/// applied in the craft's *current* frame of reference.
///
/// The mapper owns smooth *reference* signals that the controller then tracks tightly:
/// - **Rotation:** stick → target body rate, slewed by the profile's angular acceleration
///   (spin-up) and deceleration (spin-down) limits. Centered or released → attitude hold.
///   Closed stick hand → rotate back to the reset attitude.
/// - **Translation:** the throttle is a pedal. Tipping the hand down accelerates, level keeps
///   the current speed, tipping up brakes (then reverses). A closed throttle hand brakes to a
///   full stop. Speed changes are limited by the profile's linear acceleration / deceleration.
public struct IntentMapper: Sendable {
    public var profile: ControlProfile
    public var assist: FlightAssistMode
    /// Aircraft flight computer (coordinated turns, α protection, autothrottle, wings-level reset).
    public var aircraft: AircraftControl?
    /// Multirotor flight computer (velocity flying, altitude hold, hover on release).
    public var multirotor: MultirotorControl?
    /// Craft mass (kg), for multirotor thrust.
    public var mass = 1.0
    /// Attitude a closed stick hand returns the craft to (the launch attitude).
    public var resetAttitude: Quat
    /// Fraction of max forward speed / thrust available in reverse.
    public var reverseFraction = 0.25
    /// Body rate (rad/s) below which a centered stick locks in attitude hold.
    public var settleRate = 0.02

    /// Rate reference in body axes (what the controller tracks).
    private var rateReference = Vec3.zero
    private var velocityReference = Vec3.zero
    private var heldAttitude: Quat?
    private var flightPathHold: (gamma: Double, bank: Double)?
    private var autothrottlePrimed = false
    private var droneHeading: Double?
    private var droneVelocity = Vec3.zero  // world-frame velocity reference
    private var dronePosition: Vec3?       // world-frame position reference
    private var droneSpeed = 0.0           // forward-speed target (pedal)
    /// Height of the craft's centre above the ground when sitting on it (multirotor take-off check).
    public var groundClearance = 0.0

    public init(profile: ControlProfile, assist: FlightAssistMode,
                resetAttitude: Quat = Quat(ix: 0, iy: 0, iz: 0, r: 1)) {
        self.profile = profile
        self.assist = assist
        self.resetAttitude = resetAttitude
    }

    public mutating func map(_ input: PilotInput, state: RigidBodyState, air: AirData? = nil,
                             onGround: Bool = false, dt: Double) -> DesiredCraftState {
        let previousRate = rateReference
        let previousVelocity = velocityReference
        var desired: DesiredCraftState
        if multirotor != nil {
            desired = multirotorReference(input, state: state, onGround: onGround, dt: dt)
        } else if aircraft != nil {
            desired = aircraftReference(input, state: state, air: air, onGround: onGround, dt: dt)
        } else {
            desired = reference(input, state: state, dt: dt)
        }
        if dt > 0 {
            desired.angularAcceleration = (rateReference - previousRate) / dt
            if desired.assist == .velocityHold {
                desired.linearAcceleration = (velocityReference - previousVelocity) / dt
            }
        }
        return desired
    }

    private mutating func reference(_ input: PilotInput, state: RigidBodyState, dt: Double) -> DesiredCraftState {
        let bodyVelocity = state.orientation.inverse.act(state.velocity)

        // Rotation: the hand commands a *delta* — a rotation rate about the ship's own current
        // axes — so a yaw gesture yaws the ship the same way whatever its attitude. Attitude is
        // never decomposed into angles (quaternions throughout), so there is no gimbal lock.
        var targetRate = Vec3.zero  // body axes
        if input.engaged {
            for axis in 0..<3 {
                targetRate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { targetRate.x = -targetRate.x }
        } else if input.stickClosed {
            let error = HandMetrics.rotationVector(current: resetAttitude, neutral: state.orientation)
            targetRate = simd_clamp(error * profile.levelingGain, -profile.maxAngularRate, profile.maxAngularRate)
        }

        // Slew as a vector so the rotation axis never bends while rates build up; the step size
        // respects each body axis's own limit (an ellipsoid of per-axis limits).
        let slowing = simd_length(targetRate) < simd_length(rateReference) || simd_dot(targetRate, rateReference) < 0
        rateReference = Self.slew(rateReference, toward: targetRate,
                                  limits: slowing ? profile.angularDeceleration : profile.angularAcceleration, dt: dt)

        // Hold attitude once rotation has settled with nothing commanded, so a stick released
        // mid-turn decelerates to a stop instead of being yanked back.
        if input.stickClosed || simd_length(targetRate) > 0 || simd_length(rateReference) > 0 {
            heldAttitude = nil
        } else if heldAttitude == nil, simd_length(state.angularVelocity) < settleRate {
            heldAttitude = state.orientation
        }

        // Translation.
        let throttle = profile.throttleShaping.apply(input.throttle)
        let brakeStep = profile.linearDeceleration * dt
        if input.throttleClosed {
            velocityReference = Self.slew(velocityReference, toward: .zero, step: brakeStep)
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude,
                                     assist: .velocityHold, bodyVelocity: velocityReference)
        }

        switch assist {
        case .off:
            velocityReference = bodyVelocity
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude, assist: .off,
                                     thrustFraction: throttle < 0 ? throttle * reverseFraction : throttle)
        case .velocityHold:
            // Forward is -Z. Accelerate (or brake) along it; bleed off any sideways drift.
            var forwardSpeed = -velocityReference.z
            let rate = throttle * (throttle * forwardSpeed >= 0 ? profile.linearAcceleration : profile.linearDeceleration)
            forwardSpeed = (forwardSpeed + rate * dt)
                .clamped(to: -profile.maxForwardSpeed * reverseFraction...profile.maxForwardSpeed)
            let lateral = Self.slew(Vec3(velocityReference.x, velocityReference.y, 0),
                                    toward: input.translation * profile.maxStrafeSpeed, step: brakeStep)
            velocityReference = Vec3(lateral.x, lateral.y, -forwardSpeed)
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude, assist: .velocityHold,
                                     bodyVelocity: velocityReference)
        }
    }

    // MARK: - Aircraft

    /// Aircraft intent. Same gestures, flown through a fly-by-wire layer:
    /// - stick deltas command body rates, with **coordinated-turn feed-forward** (in a bank of φ the
    ///   aircraft turns at g·tanφ/V about world up — the yaw and pull a pilot would add by hand);
    /// - released stick **holds flight-path angle and bank** (heading is free to follow the turn),
    ///   so letting go means "keep doing this" — level flight stays level as speed changes;
    /// - **α protection** pitches down before the stall;
    /// - closed stick: **wings level, level flight** at the current heading;
    /// - pedal drives an **autothrottle** airspeed target; closed throttle hand: idle and brakes.
    private mutating func aircraftReference(_ input: PilotInput, state: RigidBodyState, air: AirData?,
                                            onGround: Bool, dt: Double) -> DesiredCraftState {
        guard let aircraft else { return reference(input, state: state, dt: dt) }
        let q = state.orientation
        let airspeed = air?.airspeed ?? 0
        let flying = !onGround && airspeed > aircraft.coordinationSpeed

        // Coordinated turn rate about world up for the current bank.
        var coordination = Vec3.zero
        if flying {
            let turnRate = 9.81 * tan(Self.bank(q).clamped(to: -1.3...1.3)) / airspeed
            coordination = q.inverse.act(Vec3(0, -turnRate, 0))
        }

        var pilotRate = Vec3.zero
        if input.engaged {
            for axis in 0..<3 {
                pilotRate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { pilotRate.x = -pilotRate.x }
        }
        let slowing = simd_length(pilotRate) < simd_length(rateReference) || simd_dot(pilotRate, rateReference) < 0
        rateReference = Self.slew(rateReference, toward: pilotRate,
                                  limits: slowing ? profile.angularDeceleration : profile.angularAcceleration, dt: dt)

        // α protection acts after smoothing so it bites immediately and without chatter. It looks
        // ahead by the current pitch rate so a hard pull doesn't overshoot the limit.
        var protecting = false
        if let alpha = air?.alpha, flying {
            let predicted = alpha + aircraft.alphaLookahead * state.angularVelocity.x
            if predicted > aircraft.alphaLimit {
                rateReference.x = min(rateReference.x, -aircraft.alphaProtectionGain * (predicted - aircraft.alphaLimit))
                protecting = true
            }
        }

        // Flight-path / bank hold (released stick) and wings-level recovery (closed stick).
        var attitude: Quat?
        if input.stickClosed {
            flightPathHold = (gamma: 0, bank: 0)
            attitude = flightPathAttitude(q, gamma: 0, bank: 0, alpha: air?.alpha ?? 0)
        } else if protecting || simd_length(pilotRate) > 0 || simd_length(rateReference) > 1e-3 {
            flightPathHold = nil
        } else {
            if flightPathHold == nil, simd_length(state.angularVelocity - coordination) < settleRate * 5 {
                // Airborne: hold the flight path. On the ground: hold the current pitch, so a pilot
                // can rotate and let go.
                let gamma = flying
                    ? asin((state.velocity.y / max(airspeed, 1)).clamped(to: -1...1))
                    : asin(q.act(BodyAxis.forward).y.clamped(to: -1...1))
                var bank = Self.bank(q)
                if abs(bank) < 0.05 { bank = 0 }  // snap near-level to wings level
                flightPathHold = (gamma, bank)
            }
            if let hold = flightPathHold {
                attitude = flightPathAttitude(q, gamma: flying ? hold.gamma : 0, bank: hold.bank, alpha: flying ? air?.alpha ?? 0 : 0)
            }
        }
        heldAttitude = attitude

        // Autothrottle: the pedal moves the airspeed target; it may only lead reality a little.
        if !autothrottlePrimed {
            velocityReference = Vec3(0, 0, -airspeed)
            autothrottlePrimed = true
        }
        let throttle = profile.throttleShaping.apply(input.throttle)
        var target = -velocityReference.z
        if input.throttleClosed {
            target = max(0, target - profile.linearDeceleration * dt)
        } else {
            target += throttle * (throttle >= 0 ? profile.linearAcceleration : profile.linearDeceleration) * dt
        }
        target = target.clamped(to: max(0, airspeed - aircraft.speedLead)...min(aircraft.maxAirspeed, airspeed + aircraft.speedLead))
        velocityReference = Vec3(0, 0, -target)

        var desired = DesiredCraftState(angularRate: rateReference + coordination, attitude: attitude,
                                        assist: .velocityHold, bodyVelocity: velocityReference)
        desired.thrustAxisOnly = true
        desired.brakes = input.throttleClosed
        return desired
    }

    // MARK: - Multirotor

    /// Multirotor intent, flown with **the same controls as the jet**:
    /// - pedal → target forward speed (down = faster, level = hold, up = slower);
    /// - stick pitch → climb / descend (pull back on the pad to lift off — the jet's rotation);
    /// - stick roll → banked turn, capped like a coordinated turn at g·tan(max tilt)/V;
    /// - stick yaw → flat turn;
    /// - released stick → hold altitude, heading and speed; closed stick → level flight;
    /// - closed throttle hand → brake to a hover.
    ///
    /// This is not a rotor-speed mapping. The intent becomes a **desired trajectory** — a velocity
    /// reference plus a position reference integrated from it — and the craft is steered onto it
    /// with position and velocity feedback. The resulting acceleration becomes a thrust vector; the
    /// craft tilts its rotor disc onto that vector (quaternions only) and sets collective thrust,
    /// and the allocator turns that wrench into the four individual rotor commands.
    private mutating func multirotorReference(_ input: PilotInput, state: RigidBodyState,
                                              onGround: Bool, dt: Double) -> DesiredCraftState {
        guard let mr = multirotor else { return reference(input, state: state, dt: dt) }
        let g = 9.81
        let q = state.orientation
        var heading = droneHeading ?? Self.heading(q)

        // --- Intent → trajectory (speed, climb, turn) -------------------------------------
        let shaped = input.engaged
            ? Vec3(profile.rotationShaping.apply(input.rotation.x),
                   profile.rotationShaping.apply(input.rotation.y),
                   profile.rotationShaping.apply(input.rotation.z))
            : .zero

        // Speed: the pedal moves the forward-speed target, like the jet's autothrottle.
        let pedal = profile.throttleShaping.apply(input.throttle)
        var speed = droneSpeed
        if input.throttleClosed {
            speed = max(0, speed - profile.linearDeceleration * dt)
        } else {
            speed += pedal * (pedal >= 0 ? profile.linearAcceleration : profile.linearDeceleration) * dt
        }
        speed = speed.clamped(to: 0...mr.maxHorizontalSpeed)

        // Climb: stick pitch. Descending onto the ground just settles the craft.
        var climb = shaped.x >= 0 ? shaped.x * mr.maxClimbRate : shaped.x * mr.maxDescentRate
        if input.throttleClosed { climb = 0 }

        // Turn: roll banks into a coordinated turn (limited by how far the craft can tilt at
        // this speed); yaw adds a flat turn.
        let coordinatedLimit = g * tan(mr.maxTilt) / max(speed, 1)
        let turnLimit = min(mr.maxYawRate, coordinatedLimit + (speed < 3 ? mr.maxYawRate : 0))
        var turn = shaped.z * turnLimit + shaped.y * mr.maxYawRate
        if input.stickClosed { turn = 0; climb = 0 }
        turn = turn.clamped(to: -mr.maxYawRate...mr.maxYawRate)

        // On the pad the craft can't roll forward: speed only builds once it has lifted off.
        let airborne = !onGround && state.position.y > groundClearance + 1.0
        if !airborne { speed = 0 }
        droneSpeed = speed

        // Smooth the turn rate and keep the heading target close to the craft (no wind-up).
        let slowingTurn = abs(turn) < abs(rateReference.y) || turn * rateReference.y < 0
        let turnStep = (slowingTurn ? profile.angularDeceleration.y : profile.angularAcceleration.y) * dt
        rateReference.y += (turn - rateReference.y).clamped(to: -turnStep...turnStep)
        let actualHeading = Self.heading(q)
        heading += rateReference.y * dt
        heading = actualHeading + remainder(heading - actualHeading, 2 * .pi).clamped(to: -0.3...0.3)
        droneHeading = heading

        // --- Trajectory → desired velocity and position --------------------------------------
        let forward = Vec3(-sin(heading), 0, -cos(heading))
        let targetVelocity = forward * speed + Vec3(0, climb, 0)
        let previous = droneVelocity
        droneVelocity = Self.slew(droneVelocity, toward: targetVelocity,
                                  step: (simd_length(targetVelocity) > simd_length(droneVelocity)
                                         ? profile.linearAcceleration : profile.linearDeceleration) * dt)
        let feedForward = dt > 0 ? (droneVelocity - previous) / dt : .zero

        // Desired position follows the velocity reference; it is kept within a few metres of the
        // craft so a long disturbance can't wind it up. When climb is commanded off, altitude is
        // held exactly by the position loop.
        var target = (dronePosition ?? state.position) + droneVelocity * dt
        if !airborne && climb <= 0 { target = state.position }
        // Braking (closed throttle hand) means "stop wherever the stop ends": the position
        // reference follows the craft until the speed target reaches zero, then locks.
        if input.throttleClosed, simd_length(droneVelocity) > 0.05 { target = state.position + droneVelocity * dt }
        let error = target - state.position
        if simd_length(error) > mr.maxPositionError {
            target = state.position + simd_normalize(error) * mr.maxPositionError
        }
        dronePosition = target

        // --- Position + velocity feedback → acceleration → thrust vector --------------------
        var accel = feedForward
            + (droneVelocity - state.velocity) * mr.velocityGain
            + (target - state.position) * mr.positionGain
        let maxHorizontal = g * tan(mr.maxTilt)
        let horizontal = Vec3(accel.x, 0, accel.z)
        if simd_length(horizontal) > maxHorizontal {
            let h = simd_normalize(horizontal) * maxHorizontal
            accel.x = h.x
            accel.z = h.z
        }
        var thrust = (accel + Vec3(0, g, 0)) * mass
        thrust.y = max(thrust.y, 0.25 * mass * g)

        // --- Thrust vector → attitude + collective (→ allocator → rotors) -----------------
        let up = simd_normalize(thrust)
        let tilt = Quat(from: Vec3(0, 1, 0), to: up)
        let attitude = (tilt * Quat(angle: heading, axis: BodyAxis.up)).normalized
        var collective = max(0, simd_dot(thrust, q.act(BodyAxis.up)))
        if onGround, climb <= 0.05 { collective = 0.3 * mass * g }  // idle on the pad

        var desired = DesiredCraftState(angularRate: q.inverse.act(Vec3(0, rateReference.y, 0)), attitude: attitude,
                                        assist: .velocityHold, bodyVelocity: .zero)
        desired.bodyForce = Vec3(0, collective, 0)
        desired.brakes = input.throttleClosed
        return desired
    }

    /// Heading (rad, right-hand about world up; 0 = facing −Z) of an attitude's forward axis,
    /// falling back to its up axis when pointing straight up or down.
    static func heading(_ q: Quat) -> Double {
        var forward = q.act(BodyAxis.forward)
        if abs(forward.y) > 0.98 { forward = -q.act(BodyAxis.up) * (forward.y > 0 ? 1 : -1) }
        return atan2(-forward.x, -forward.z)
    }

    /// Attitude that flies flight-path angle `gamma` at bank `bank` on the current heading:
    /// nose above the flight path by the current angle of attack. Built by composing quaternions;
    /// near vertical (heading undefined) it falls back to the current attitude.
    private func flightPathAttitude(_ q: Quat, gamma: Double, bank: Double, alpha: Double) -> Quat {
        var forward = q.act(BodyAxis.forward)
        forward.y = 0
        guard simd_length(forward) > 0.2 else { return q }
        let f = simd_normalize(forward)
        let heading = atan2(-f.x, -f.z)
        return (Quat(angle: heading, axis: BodyAxis.up)
                * Quat(angle: gamma + alpha.clamped(to: -0.1...0.25), axis: BodyAxis.right)
                * Quat(angle: -bank, axis: Vec3(0, 0, 1))).normalized
    }

    /// Bank angle (rad, positive = right wing down) from the body axes.
    static func bank(_ q: Quat) -> Double {
        let right = q.act(BodyAxis.right), up = q.act(BodyAxis.up)
        return atan2(-right.y, up.y)
    }

    /// Vector slew whose maximum step along any direction lies on the ellipsoid of per-axis
    /// limits — the direction of change is preserved exactly.
    static func slew(_ value: Vec3, toward target: Vec3, limits: Vec3, dt: Double) -> Vec3 {
        let delta = target - value
        let distance = simd_length(delta)
        guard distance > 0 else { return target }
        let d = delta / distance
        let limit = 1 / simd_length(d / limits)
        return slew(value, toward: target, step: limit * dt)
    }

    static func slew(_ value: Vec3, toward target: Vec3, step: Double) -> Vec3 {
        let delta = target - value
        let distance = simd_length(delta)
        return distance <= step ? target : value + delta * (step / distance)
    }
}
