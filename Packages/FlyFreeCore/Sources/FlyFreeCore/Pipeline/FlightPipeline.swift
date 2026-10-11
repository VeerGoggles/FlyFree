import simd

/// Everything that happened in one simulation frame — for HUDs, visuals and debugging.
public struct FlightTelemetry: Sendable {
    public var time: Double
    public var input: PilotInput
    public var desired: DesiredCraftState
    public var demand: Wrench
    public var achieved: Wrench
    /// Actuator commands, index-aligned with `craft.actuators`.
    public var actuatorCommands: [Double]
    public var state: RigidBodyState
    /// Air-relative flight condition (nil in vacuum).
    public var air: AirData?
    public var wheelsOnGround = 0
    /// Lift as a multiple of weight (g), aircraft only.
    public var loadFactor = 0.0

    /// Speed relative to the starting frame (m/s).
    public var speed: Double { simd_length(state.velocity) }
    public var onGround: Bool { wheelsOnGround > 0 }
}

/// The full hands → craft pipeline:
///
///     HandFrame ─► HandPlane (least-squares fit through the finger joints)
///               ─► PilotInputInterpreter ─► PilotInput (relative to calibrated neutral)
///               ─► IntentMapper          ─► DesiredCraftState
///               ─► FlightController      ─► Wrench demand
///               ─► ControlAllocator      ─► actuator commands (+ achieved wrench)
///               ─► RigidBodyIntegrator   ─► RigidBodyState
///
/// Input and intent run once per rendered frame; control, allocation and physics run at a
/// fixed sub-step so behaviour does not depend on frame rate. Aircraft add an aerodynamic
/// model, landing gear and q̄-scaled control surfaces inside the same loop.
public struct FlightPipeline: Sendable {
    public let craft: CraftDefinition
    public var interpreter: PilotInputInterpreter
    public var mapper: IntentMapper
    public var controller: FlightController
    public var allocator: ControlAllocator
    public var environment: any FlightEnvironment
    public var state: RigidBodyState
    public var physicsStep = 1.0 / 180.0

    private var time = 0.0
    private var accumulator = 0.0
    private var wheelsOnGround = 0
    private var lastLift = 0.0

    public init(
        craft: CraftDefinition,
        interpreter: InterpreterConfiguration = .init(),
        calibration: HandCalibration? = nil,
        assist: FlightAssistMode = .velocityHold,
        environment: (any FlightEnvironment)? = nil,
        initialState: RigidBodyState? = nil
    ) {
        let start = initialState ?? craft.startState
        self.craft = craft
        self.interpreter = PilotInputInterpreter(configuration: interpreter, calibration: calibration)
        self.mapper = IntentMapper(profile: craft.profile, assist: assist, resetAttitude: start.orientation)
        self.mapper.aircraft = craft.aircraft
        self.controller = FlightController(gains: craft.gains, mass: craft.mass,
                                           maxForwardThrust: craft.maxForwardThrust,
                                           maxHoldRate: craft.profile.maxAngularRate * 0.25)
        self.allocator = ControlAllocator(actuators: craft.actuators, weights: craft.allocationWeights)
        self.environment = environment ?? craft.environment
        self.state = start
    }

    /// Sets the neutral hand frames that all stick and lever input is measured against.
    public mutating func calibrate(_ calibration: HandCalibration) {
        interpreter.calibration = calibration
    }

    /// Advances the simulation by a rendered frame of length `dt` seconds.
    public mutating func step(left: HandFrame?, right: HandFrame?, dt: Double) -> FlightTelemetry {
        let frameDt = dt.clamped(to: 0...0.1)
        let input = interpreter.update(left: left, right: right, time: time + frameDt)
        return step(input: input, dt: frameDt)
    }

    public var airData: AirData? {
        environment.air(at: state.position).map { AirData.compute(state: state, air: $0) }
    }

    /// Advances the simulation from an already-interpreted `PilotInput` (useful for
    /// alternative input devices, replays and tests).
    public mutating func step(input: PilotInput, dt: Double) -> FlightTelemetry {
        let frameDt = dt.clamped(to: 0...0.1)
        time += frameDt
        accumulator += frameDt
        let desired = mapper.map(input, state: state, air: airData, onGround: wheelsOnGround > 0, dt: frameDt)

        var demand = Wrench.zero
        var achieved = allocator.achievedWrench
        while accumulator >= physicsStep {
            let air = airData
            var airframe: AerodynamicModel.Result?
            if let air, let aero = craft.aerodynamics {
                airframe = aero.loads(air, angularVelocity: state.angularVelocity, speedBrake: desired.brakes)
            }
            demand = controller.demand(desired: desired, state: state,
                                       airframeTorque: airframe?.bodyTorque ?? .zero,
                                       bodyAirVelocity: craft.aircraft != nil ? air?.bodyAirVelocity : nil)
            let q̄ = air?.dynamicPressure ?? 0
            let effectiveness = craft.actuators.map { $0.kind == .aeroSurface ? q̄ : 1 }
            achieved = allocator.allocate(demand, effectiveness: effectiveness)

            var loads = environment.loads(on: state, mass: craft.mass)
            if let airframe {
                loads.worldForce += state.orientation.act(airframe.bodyForce)
                loads.bodyTorque += airframe.bodyTorque
                lastLift = airframe.lift
            }
            if let gear = craft.landingGear, let ground = environment.groundHeight {
                let contact = gear.loads(state: state, groundHeight: ground, brakes: desired.brakes)
                loads = loads + contact.loads
                wheelsOnGround = contact.wheelsOnGround
            }
            RigidBodyIntegrator.step(&state, mass: craft.mass, bodyWrench: achieved, external: loads, dt: physicsStep)
            accumulator -= physicsStep
        }

        var telemetry = FlightTelemetry(
            time: time, input: input, desired: desired, demand: demand, achieved: achieved,
            actuatorCommands: allocator.currentCommands, state: state
        )
        telemetry.air = airData
        telemetry.wheelsOnGround = wheelsOnGround
        if craft.aerodynamics != nil { telemetry.loadFactor = lastLift / (craft.mass.mass * 9.81) }
        return telemetry
    }
}
