import Testing
import simd
@testable import FlyFreeCore

/// Drone flight, RC angle mode: throttle-hand height is the collective (centre = hover thrust),
/// stick-hand tilt tilts the drone, twist turns it.
@Suite struct DroneTests {
    let drone = CraftCatalog.drone
    let dt = 1.0 / 90

    private func run(_ p: inout FlightPipeline, _ input: PilotInput, seconds: Double) -> FlightTelemetry {
        var t = p.step(input: input, dt: dt)
        for _ in 1..<Int(seconds * 90) { t = p.step(input: input, dt: dt) }
        return t
    }

    /// Throttle hand raised/lowered (normalized height) and the stick hand's normalized rotation.
    private func hands(up: Double = 0, stick: Vec3 = .zero) -> PilotInput {
        PilotInput(rotation: stick, translation: Vec3(0, up, 0), engaged: true)
    }

    /// Lifted off, then stopped climbing with the hover assist.
    private func airborne() -> FlightPipeline {
        var p = FlightPipeline(craft: drone)
        _ = run(&p, hands(up: 0.5), seconds: 3)
        _ = run(&p, PilotInput(engaged: true, throttleClosed: true), seconds: 4)
        return p
    }

    private func tilt(_ t: FlightTelemetry) -> Double {
        acos(t.state.orientation.act(BodyAxis.up).y.clamped(to: -1...1))
    }

    @Test func rotorsMixLikeAQuadcopter() {
        var allocator = ControlAllocator(actuators: drone.actuators, weights: drone.allocationWeights)
        let hover = Wrench(force: Vec3(0, drone.mass.mass * 9.81, 0))
        allocator.allocate(hover)
        for c in allocator.currentCommands { #expect(abs(c - 0.44) < 0.01) }
        let achieved = allocator.allocate(hover + Wrench(torque: Vec3(0, 200, 0)))
        #expect(abs(achieved.force.y - hover.force.y) < 20)
        #expect(achieved.torque.y > 190)
        let c = allocator.currentCommands
        #expect(abs(c[0] - c[2]) < 0.01 && abs(c[1] - c[3]) < 0.01 && abs(c[0] - c[1]) > 0.1)
    }

    @Test func startsFlatWithTheThrottleHandDrivenByHeight() {
        #expect(drone.startingPose == .bothFlat)
        #expect(drone.throttleHand == .position)
    }

    @Test func handHeightIsRawCollectiveCentredOnHover() {
        let hover = drone.mass.mass * 9.81
        var p = airborne()
        // Centred: exactly hover thrust, so vertical speed holds (no altitude hold pulls it back).
        p.state.velocity.y = 1
        var t = run(&p, hands(), seconds: 2)
        #expect(abs(t.achieved.force.y - hover) < 0.02 * hover)
        #expect(abs(t.state.velocity.y - 1) < 0.1)
        // Full up: the rotors' top collective; full down: none.
        t = run(&p, hands(up: 1), seconds: 0.5)
        #expect(abs(t.desired.bodyForce!.y - drone.maxRotorThrust * drone.multirotor!.maxCollectiveFraction) < 1)
        t = run(&p, hands(up: -1), seconds: 0.1)
        #expect(t.desired.bodyForce!.y < 1)
    }

    @Test func raisingTheHandLiftsOffAndLoweringItComesBackDown() {
        var p = FlightPipeline(craft: drone)
        var t = run(&p, hands(), seconds: 2)
        #expect(t.wheelsOnGround == 4)   // centred hand on the pad: idle
        t = run(&p, hands(up: 0.5), seconds: 3)
        #expect(!t.onGround)
        #expect(t.state.velocity.y > 3)
        t = run(&p, hands(up: -0.5), seconds: 5)
        #expect(t.state.velocity.y < 0)
    }

    @Test func tippingTheStickHandTiltsTheDroneAndFliesForward() {
        var p = airborne()
        // Fingertips down 30° (1/3 of full) → the nose tips down 0.6 × 30° and it flies forward.
        let t = run(&p, hands(up: 0.1, stick: Vec3(-1.0 / 3, 0, 0)), seconds: 4)
        let expected = mrTilt(1.0 / 3)
        #expect(abs(tilt(t) - expected) < 0.03)
        #expect(t.state.orientation.act(BodyAxis.forward).y < 0)  // nose down
        #expect(-t.state.velocity.z > 5)
    }

    @Test func tiltIsCappedAtTheAngleLimit() {
        var p = airborne()
        let t = run(&p, hands(up: 0.2, stick: Vec3(-1, 0, -1)), seconds: 3)
        #expect(abs(tilt(t) - drone.multirotor!.maxTilt) < 0.03)
    }

    @Test func rollingRightSlidesRightAndTwistingTurns() {
        var p = airborne()
        var t = run(&p, hands(up: 0.05, stick: Vec3(0, 0, -0.3)), seconds: 4)
        #expect(t.state.velocity.x > 4)
        #expect(abs(t.state.velocity.z) < 1)
        p = airborne()
        t = run(&p, hands(stick: Vec3(0, 1, 0)), seconds: 3)
        #expect(IntentMapper.heading(t.state.orientation) > 1.0)
    }

    @Test func aFlatStickHandLevelsTheDroneWhichThenCoasts() {
        var p = airborne()
        _ = run(&p, hands(up: 0.1, stick: Vec3(-0.4, 0, 0)), seconds: 3)
        let t = run(&p, hands(), seconds: 2)
        #expect(tilt(t) < 0.02)
        #expect(-t.state.velocity.z > 5)   // angle mode: no position hold
    }

    @Test func closedHandsHoverAndTurnHome() {
        var p = airborne()
        _ = run(&p, hands(up: 0.6, stick: Vec3(0, 0.7, 0)), seconds: 3)
        var t = run(&p, PilotInput(engaged: true, throttleClosed: true), seconds: 6)
        #expect(abs(t.state.velocity.y) < 0.1)
        t = run(&p, PilotInput(stickClosed: true, throttleClosed: true), seconds: 6)
        #expect(abs(IntentMapper.heading(t.state.orientation)) < 0.03)
        #expect(tilt(t) < 0.02)
    }

    @Test func stickTiltCannotTipItOnThePad() {
        var p = FlightPipeline(craft: drone)
        let t = run(&p, hands(stick: Vec3(-1, 0, -1)), seconds: 3)
        #expect(t.wheelsOnGround == 4)
        #expect(simd_length(t.state.velocity) < 0.05)
    }

    private func mrTilt(_ normalized: Double) -> Double {
        let mr = drone.multirotor!
        return min(mr.tiltShaping.apply(normalized) * .pi / 2 * mr.tiltPerHandTilt, mr.maxTilt)
    }
}

/// The interpreter turns the throttle hand's height (relative to its calibrated spot) into the
/// collective command.
@Suite struct HandPositionInputTests {
    private let flat = Quat(ix: 0, iy: 0, iz: 0, r: 1)

    private func frames(leftAt offset: Vec3) -> (HandFrame, HandFrame) {
        (SyntheticHand.make(chirality: .left, orientation: flat, position: Vec3(-0.2, 1.0, -0.35) + offset),
         SyntheticHand.make(chirality: .right, orientation: flat, position: Vec3(0.2, 1.0, -0.35)))
    }

    @Test func calibrationCapturesHandPositions() throws {
        var session = CalibrationSession(pose: .bothFlat)
        var progress = CalibrationProgress(left: .notDetected, right: .notDetected, fraction: 0)
        var t = 0.0
        while progress.result == nil, t < 3 {
            let (l, r) = frames(leftAt: .zero)
            progress = session.update(left: l, right: r, time: t)
            t += 1.0 / 90
        }
        let result = try #require(progress.result)
        let home = try #require(result.leftPosition)
        let plane = try #require(HandMetrics.plane(of: frames(leftAt: .zero).0))
        #expect(simd_distance(home, plane.centroid) < 1e-9)
    }

    @Test func handHeightBecomesTheCollectiveCommand() throws {
        let (l0, r0) = frames(leftAt: .zero)
        let home = try #require(HandMetrics.plane(of: l0)).centroid
        var config = InterpreterConfiguration()
        config.throttleHandMode = .position
        var interpreter = PilotInputInterpreter(
            configuration: config,
            calibration: HandCalibration(left: flat, right: flat, leftPosition: home,
                                         rightPosition: try #require(HandMetrics.plane(of: r0)).centroid))
        var input = PilotInput()
        // 7.5 cm up: half; moving it forward does nothing.
        for i in 0..<120 {
            let (l, r) = frames(leftAt: Vec3(0, 0.075, -0.15))
            input = interpreter.update(left: l, right: r, time: Double(i) / 90)
        }
        #expect(abs(input.translation.y - 0.5) < 0.02)
        #expect(input.translation.z == 0)
        #expect(input.throttle == 0)
    }
}
