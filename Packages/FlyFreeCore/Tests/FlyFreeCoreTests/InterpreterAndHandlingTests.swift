import Testing
import simd
@testable import FlyFreeCore

private let fist: [Finger: Double] = [.index: 0.9, .middle: 0.9, .ring: 0.9, .little: 0.9]

private func hand(_ c: Chirality, yaw: Double = 0, pitch: Double = 0, roll: Double = 0, curl: [Finger: Double] = [:]) -> HandFrame {
    SyntheticHand.make(chirality: c, orientation: .fromYawPitchRoll(yaw: yaw, pitch: pitch, roll: roll),
                       position: Vec3(c == .right ? 0.2 : -0.2, 1.0, -0.35), curl: curl)
}

@Suite struct InterpreterTests {
    @Test func flatHandFliesClosedHandResets() {
        var interpreter = PilotInputInterpreter(calibration: .identity)
        var input = PilotInput()
        for i in 0..<90 {
            input = interpreter.update(left: hand(.left), right: hand(.right, yaw: -0.25, pitch: 0.2), time: Double(i) / 90)
        }
        #expect(input.engaged)
        #expect(input.rotation.x > 0.35)   // fingertips up → pitch up
        #expect(input.rotation.y < -0.35)  // wrist turned right → yaw right
        #expect(abs(input.rotation.z) < 0.15)

        input = interpreter.update(left: hand(.left), right: hand(.right, curl: fist), time: 1.1)
        #expect(!input.engaged)
        #expect(input.stickClosed)
        #expect(input.rotation == .zero)
    }

    @Test func noCalibrationNoStick() {
        var interpreter = PilotInputInterpreter()
        let input = interpreter.update(left: nil, right: hand(.right, pitch: 0.3), time: 0)
        #expect(!input.engaged)
    }

    @Test func throttleIsAPedal() {
        var interpreter = PilotInputInterpreter(calibration: .identity)
        var input = PilotInput()
        var t = 0.0
        func run(_ left: HandFrame?, seconds: Double = 1) {
            for _ in 0..<Int(seconds * 90) {
                t += 1.0 / 90
                input = interpreter.update(left: left, right: hand(.right), time: t)
            }
        }
        run(hand(.left, pitch: -0.45))  // fingertips pushed down → accelerate
        #expect(input.throttle > 0.95)
        run(hand(.left, pitch: 0.2))    // fingertips up → brake / reverse
        #expect(input.throttle < -0.35)
        run(hand(.left, pitch: -0.3, curl: fist))
        #expect(input.throttleClosed)
        #expect(input.throttle == 0)

        // Lose the hand: zeroed after the grace period.
        run(hand(.left, pitch: -0.3))
        run(nil, seconds: 0.8)
        #expect(input.throttle == 0)
    }
}

@Suite struct HandlingTests {
    let craft = CraftCatalog.genericSpacecraft

    @Test func spinUpIsGentleAndSpinDownIsQuick() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        let full = PilotInput(rotation: Vec3(1, 0, 0), engaged: true)
        var t = 0.0
        while pipeline.state.angularVelocity.x < 0.59 * 0.98, t < 5 {
            _ = pipeline.step(input: full, dt: 1 / 90); t += 1 / 90
        }
        let spinUp = t
        // 0.6 rad/s at 1.2 rad/s² ≈ 0.5 s.
        #expect(spinUp > 0.4 && spinUp < 0.7)

        t = 0
        while simd_length(pipeline.state.angularVelocity) > 0.01, t < 5 {
            _ = pipeline.step(input: PilotInput(engaged: true), dt: 1 / 90); t += 1 / 90
        }
        // 0.6 rad/s at 2.4 rad/s² ≈ 0.25 s.
        #expect(t < 0.45)
    }

    @Test func closedStickReturnsToResetAttitude() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        for _ in 0..<(90 * 2) { _ = pipeline.step(input: PilotInput(rotation: Vec3(0.8, 0.5, -0.6), engaged: true), dt: 1 / 90) }
        #expect(simd_length(HandMetrics.rotationVector(current: pipeline.state.orientation, neutral: .init(ix: 0, iy: 0, iz: 0, r: 1))) > 0.5)
        for _ in 0..<(90 * 8) { _ = pipeline.step(input: PilotInput(stickClosed: true), dt: 1 / 90) }
        let error = HandMetrics.rotationVector(current: pipeline.state.orientation, neutral: .init(ix: 0, iy: 0, iz: 0, r: 1))
        #expect(simd_length(error) < 0.01)
        #expect(simd_length(pipeline.state.angularVelocity) < 0.01)
    }

    @Test func pedalAcceleratesHoldsSpeedAndClosedHandStops() {
        var pipeline = FlightPipeline(craft: craft, assist: .velocityHold)
        var telemetry = pipeline.step(input: PilotInput(throttle: 1), dt: 1 / 90)
        for _ in 0..<(90 * 10) { telemetry = pipeline.step(input: PilotInput(throttle: 1), dt: 1 / 90) }
        #expect(abs(telemetry.speed - 40) < 2)   // 4 m/s² for 10 s

        // Level pedal: keep the speed (space has no drag).
        for _ in 0..<(90 * 5) { telemetry = pipeline.step(input: .neutral, dt: 1 / 90) }
        #expect(abs(telemetry.speed - 40) < 2)

        // Closed hand: brake at 12 m/s² → stopped in ~3.3 s.
        var t = 0.0
        while telemetry.speed > 0.2, t < 10 {
            telemetry = pipeline.step(input: PilotInput(throttleClosed: true), dt: 1 / 90); t += 1 / 90
        }
        #expect(t < 4.2)
    }

    @Test func closedThrottleAlsoStopsInNewtonianMode() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        for _ in 0..<(90 * 5) { _ = pipeline.step(input: PilotInput(throttle: 1), dt: 1 / 90) }
        var telemetry = pipeline.step(input: PilotInput(throttleClosed: true), dt: 1 / 90)
        #expect(telemetry.speed > 20)
        for _ in 0..<(90 * 10) { telemetry = pipeline.step(input: PilotInput(throttleClosed: true), dt: 1 / 90) }
        #expect(telemetry.speed < 0.2)
    }
}
