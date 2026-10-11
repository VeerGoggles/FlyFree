import Testing
import simd
@testable import FlyFreeCore

/// Drone-style flight: the throttle hand's position moves the drone up/down and forward/back
/// (additively); the stick hand turns, slides and adds fore/aft.
@Suite struct DroneTests {
    let drone = CraftCatalog.drone
    let dt = 1.0 / 90

    private func run(_ p: inout FlightPipeline, _ input: PilotInput, seconds: Double) -> FlightTelemetry {
        var t = p.step(input: input, dt: dt)
        for _ in 1..<Int(seconds * 90) { t = p.step(input: input, dt: dt) }
        return t
    }

    /// Throttle hand raised/lowered (y) and pushed forward (−z), as normalized translation.
    private func hand(up: Double = 0, forward: Double = 0, stick: Vec3 = .zero) -> PilotInput {
        PilotInput(rotation: stick, translation: Vec3(0, up, -forward), engaged: true)
    }

    private func airborne() -> FlightPipeline {
        var p = FlightPipeline(craft: drone)
        _ = run(&p, hand(up: 1), seconds: 5)
        _ = run(&p, hand(), seconds: 3)
        return p
    }

    private func horizontalSpeed(_ t: FlightTelemetry) -> Double {
        simd_length(Vec3(t.state.velocity.x, 0, t.state.velocity.z))
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

    @Test func startsFlatWithTheThrottleHandDrivenByPosition() {
        #expect(drone.startingPose == .bothFlat)
        #expect(drone.throttleHand == .position)
    }

    @Test func raisingTheHandLiftsOffAndLevelHandHoldsAltitude() {
        var p = FlightPipeline(craft: drone)
        var t = run(&p, hand(up: 1), seconds: 4)
        #expect(!t.onGround)
        #expect(t.state.velocity.y > 5)
        _ = run(&p, hand(), seconds: 3)
        let y = p.state.position.y
        t = run(&p, hand(), seconds: 5)
        #expect(abs(t.state.position.y - y) < 0.3)
        #expect(simd_length(t.state.velocity) < 0.1)
    }

    @Test func pushingTheHandForwardFliesForwardAndPullingBackFliesBack() {
        var p = airborne()
        var t = run(&p, hand(forward: 1), seconds: 8)
        #expect(-t.state.velocity.z > drone.multirotor!.maxHorizontalSpeed - 1)
        let y = p.state.position.y
        t = run(&p, hand(forward: -1), seconds: 12)
        #expect(t.state.velocity.z > drone.multirotor!.maxHorizontalSpeed - 1)
        #expect(abs(t.state.position.y - y) < 1.5)
    }

    @Test func upAndForwardCombineAdditively() {
        var p = airborne()
        let t = run(&p, hand(up: 1, forward: 1), seconds: 7)
        #expect(-t.state.velocity.z > drone.multirotor!.maxHorizontalSpeed - 1)    // full forward…
        #expect(abs(t.state.velocity.y - drone.multirotor!.maxClimbRate) < 0.5)   // …and full climb
        // The rotor disc is tilted forward and producing more than hover thrust at the same time.
        let up = t.state.orientation.act(BodyAxis.up)
        #expect(up.z < -0.02)   // tilted forward (at steady speed, just enough to beat drag)
        #expect(t.achieved.force.y > drone.mass.mass * 9.81)
    }

    @Test func stickAddsToTheHandAndSlidesAndTurns() {
        var p = airborne()
        // Hand half forward + stick tipped forward → faster than the hand alone.
        var t = run(&p, hand(forward: 0.6), seconds: 8)
        let handOnly = horizontalSpeed(t)
        t = run(&p, hand(forward: 0.6, stick: Vec3(-0.8, 0, 0)), seconds: 8)
        #expect(horizontalSpeed(t) > handOnly + 2)
        // Roll right slides right; twist left turns left.
        p = airborne()
        t = run(&p, hand(stick: Vec3(0, 0, -1)), seconds: 6)
        #expect(t.state.velocity.x > drone.multirotor!.maxHorizontalSpeed - 1)
        p = airborne()
        t = run(&p, hand(stick: Vec3(0, 1, 0)), seconds: 3)
        #expect(IntentMapper.heading(t.state.orientation) > 1.0)
    }

    @Test func releasingHoldsPositionAgainstAGust() {
        var p = airborne()
        let hold = p.state.position
        p.state.velocity += Vec3(3, -1, 2)
        let t = run(&p, hand(), seconds: 8)
        #expect(simd_distance(t.state.position, hold) < 0.5)
    }

    @Test func closedHandsHoverAndTurnHome() {
        var p = airborne()
        _ = run(&p, hand(up: 0.5, forward: 1, stick: Vec3(0, 0.7, 0)), seconds: 5)
        var t = run(&p, PilotInput(translation: Vec3(0, 1, -1), engaged: true, throttleClosed: true), seconds: 7)
        #expect(simd_length(t.state.velocity) < 0.3)
        t = run(&p, PilotInput(stickClosed: true), seconds: 6)
        #expect(abs(IntentMapper.heading(t.state.orientation)) < 0.03)
    }

    @Test func loweringTheHandLands() {
        var p = airborne()
        let t = run(&p, hand(up: -1), seconds: 10)
        #expect(t.wheelsOnGround == 4)
        #expect(abs(t.state.velocity.y) < 0.1)
    }

    @Test func stickAloneCannotDragItAcrossThePad() {
        var p = FlightPipeline(craft: drone)
        let t = run(&p, hand(forward: 1, stick: Vec3(-1, 0, -1)), seconds: 3)
        #expect(t.wheelsOnGround == 4)
        #expect(simd_length(t.state.velocity) < 0.05)
    }
}

/// The interpreter turns the throttle hand's position (relative to its calibrated spot) into
/// translation commands.
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

    @Test func handOffsetBecomesUpAndForwardCommands() throws {
        let (l0, r0) = frames(leftAt: .zero)
        let home = try #require(HandMetrics.plane(of: l0)).centroid
        var config = InterpreterConfiguration()
        config.throttleHandMode = .position
        var interpreter = PilotInputInterpreter(
            configuration: config,
            calibration: HandCalibration(left: flat, right: flat, leftPosition: home,
                                         rightPosition: try #require(HandMetrics.plane(of: r0)).centroid))
        var input = PilotInput()
        // 7.5 cm up and 15 cm forward: half climb, full forward.
        for i in 0..<120 {
            let (l, r) = frames(leftAt: Vec3(0, 0.075, -0.15))
            input = interpreter.update(left: l, right: r, time: Double(i) / 90)
        }
        #expect(abs(input.translation.y - 0.5) < 0.02)
        #expect(abs(input.translation.z - -1) < 0.02)
        #expect(input.throttle == 0)
    }
}
