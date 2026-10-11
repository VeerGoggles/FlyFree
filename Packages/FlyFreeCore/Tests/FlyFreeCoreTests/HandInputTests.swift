import Testing
import simd
@testable import FlyFreeCore

private let fist: [Finger: Double] = [.index: 0.9, .middle: 0.9, .ring: 0.9, .little: 0.9]

private func hand(_ c: Chirality, yaw: Double = 0, pitch: Double = 0, roll: Double = 0, curl: [Finger: Double] = [:]) -> HandFrame {
    SyntheticHand.make(chirality: c, orientation: .fromYawPitchRoll(yaw: yaw, pitch: pitch, roll: roll),
                       position: Vec3(c == .right ? 0.2 : -0.2, 1.0, -0.35), curl: curl)
}

/// Deterministic per-joint noise, imitating hand-tracking jitter.
private func jittered(_ frame: HandFrame, amplitude: Double, seed: UInt64) -> HandFrame {
    var state = seed
    func noise() -> Double {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return (Double(state >> 11) / Double(1 << 53) * 2 - 1) * amplitude
    }
    var out = frame
    for (finger, chain) in frame.fingers {
        out.fingers[finger] = chain.map { $0 + Vec3(noise(), noise(), noise()) }
    }
    return out
}

@Suite struct HandPlaneTests {
    @Test func straightFingerHasNoFlexion() {
        #expect(HandMetrics.flexion(hand(.right), .index) < 0.01)
    }

    @Test(arguments: Chirality.allCases)
    func flatHandPlaneIsLevelWithTinyResidual(chirality: Chirality) throws {
        let plane = try #require(HandMetrics.plane(of: hand(chirality)))
        #expect(plane.rmsResidual < 1e-6)
        #expect(simd_length(HandMetrics.rotationVector(current: plane.orientation, neutral: HandCalibration.identity[chirality])) < 1e-6)
    }

    @Test func fistIsNotFlat() throws {
        let plane = try #require(HandMetrics.plane(of: hand(.right, curl: fist)))
        #expect(plane.rmsResidual > 0.012)
        #expect(HandMetrics.curl(hand(.right, curl: fist)) > 0.5)
    }

    @Test(arguments: Chirality.allCases)
    func rotationVectorSeparatesAxes(chirality: Chirality) throws {
        let cases: [(Double, Double, Double, Int)] = [(0, 0.3, 0, 0), (0.3, 0, 0, 1), (0, 0, 0.3, 2)]
        for (yaw, pitch, roll, axis) in cases {
            let plane = try #require(HandMetrics.plane(of: hand(chirality, yaw: yaw, pitch: pitch, roll: roll)))
            let rv = HandMetrics.rotationVector(current: plane.orientation, neutral: .init(ix: 0, iy: 0, iz: 0, r: 1))
            #expect(abs(rv[axis] - 0.3) < 1e-6)
            #expect(abs(simd_length(rv) - 0.3) < 1e-6)
        }
    }

    @Test func planeFitIsMoreRobustThanASingleFinger() throws {
        // Under 4 mm of joint jitter, the plane's pitch error is well below that of the index finger alone.
        var planeError = 0.0, fingerError = 0.0
        for seed in 1...200 as ClosedRange<UInt64> {
            let noisy = jittered(hand(.right), amplitude: 0.004, seed: seed)
            let plane = try #require(HandMetrics.plane(of: noisy))
            planeError += abs(asin(plane.forward.y))
            let chain = noisy.fingers[.index]!
            fingerError += abs(asin(simd_normalize(chain.last! - chain[1]).y))
        }
        #expect(planeError < fingerError * 0.6)
    }
}

@Suite struct CalibrationTests {
    @Test func completesAfterSteadyFlatHold() throws {
        var session = CalibrationSession()
        let tilt = 0.1
        var progress = session.update(left: hand(.left, yaw: tilt), right: hand(.right, yaw: -tilt), time: 0)
        #expect(progress.left == .ready && progress.right == .ready)
        var t = 0.0
        while progress.result == nil, t < 3 {
            t += 1.0 / 90
            progress = session.update(left: hand(.left, yaw: tilt), right: hand(.right, yaw: -tilt), time: t)
        }
        let result = try #require(progress.result)
        #expect(abs(t - 1.5) < 0.05)
        #expect(abs(HandMetrics.rotationVector(current: result.left, neutral: .fromYawPitchRoll(yaw: tilt, pitch: 0, roll: 0)).y) < 1e-6)
        #expect(abs(HandMetrics.rotationVector(current: result.right, neutral: .fromYawPitchRoll(yaw: -tilt, pitch: 0, roll: 0)).y) < 1e-6)
    }

    @Test func rejectsFistsTiltedPalmsMissingHandsAndMotion() {
        var session = CalibrationSession()
        #expect(session.update(left: nil, right: hand(.right), time: 0).left == .notDetected)
        #expect(session.update(left: hand(.left, curl: fist), right: hand(.right), time: 0.01).left == .notFlat)
        #expect(session.update(left: hand(.left, roll: 1.4), right: hand(.right), time: 0.02).left == .notLevel)
        _ = session.update(left: hand(.left), right: hand(.right), time: 0.03)
        let moving = session.update(left: hand(.left, yaw: 0.2), right: hand(.right), time: 0.04)
        #expect(moving.left == .moving)
        #expect(moving.fraction == 0)
    }
}

@Suite struct InterpreterTests {
    @Test func flatHandFliesAndFistReleases() {
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
        #expect(input.rotation == .zero)
    }

    @Test func noCalibrationNoStick() {
        var interpreter = PilotInputInterpreter()
        let input = interpreter.update(left: nil, right: hand(.right, pitch: 0.3), time: 0)
        #expect(input.dominantHandTracked)
        #expect(!input.engaged)
    }

    @Test func leverThrottleForwardReverseAndLock() {
        var interpreter = PilotInputInterpreter(calibration: .identity)
        var input = PilotInput()
        var t = 0.0
        func run(_ left: HandFrame?, seconds: Double = 1) {
            for _ in 0..<Int(seconds * 90) {
                t += 1.0 / 90
                input = interpreter.update(left: left, right: hand(.right), time: t)
            }
        }
        run(hand(.left, pitch: 0.45))
        #expect(input.throttle > 0.95)
        run(hand(.left, pitch: -0.2))
        #expect(input.throttle < -0.35)

        // Curl to lock cruise, then wave the closed hand around: throttle stays put.
        run(hand(.left, pitch: 0.3))
        let cruise = input.throttle
        run(hand(.left, pitch: -0.4, curl: fist))
        #expect(input.throttleLocked)
        #expect(abs(input.throttle - cruise) < 0.05)

        // Lose the hand: held for the grace period, then zeroed.
        run(nil, seconds: 0.3)
        #expect(abs(input.throttle - cruise) < 0.05)
        run(nil, seconds: 0.5)
        #expect(input.throttle == 0)
    }
}
