import Testing
import simd
@testable import FlyFreeCore

/// Per-simulation starting poses: calibration only completes in the craft's pose, and controls
/// are measured in the pilot's frame whatever pose the hands started in.
@Suite struct StartingPoseTests {
    private let identity = Quat(ix: 0, iy: 0, iz: 0, r: 1)
    /// Right hand thumb up (palm facing left / in); left hand thumb up (palm facing right / in).
    private let rightThumbUp = Quat(angle: -.pi / 2, axis: Vec3(0, 0, 1))
    private let leftThumbUp = Quat(angle: .pi / 2, axis: Vec3(0, 0, 1))

    private func hand(_ c: Chirality, _ q: Quat, curl: Double = 0) -> HandFrame {
        SyntheticHand.make(chirality: c, orientation: q, position: Vec3(c == .right ? 0.2 : -0.2, 1.0, -0.35),
                           curl: [.index: curl, .middle: curl, .ring: curl, .little: curl])
    }

    private func calibrate(_ pose: StartingPose, left: Quat, right: Quat, dominant: Chirality = .right) -> CalibrationProgress {
        var session = CalibrationSession(pose: pose, dominant: dominant)
        var progress = session.update(left: hand(.left, left), right: hand(.right, right), time: 0)
        var t = 0.0
        while progress.result == nil, t < 2.5 {
            t += 1.0 / 90
            progress = session.update(left: hand(.left, left), right: hand(.right, right), time: t)
        }
        return progress
    }

    @Test func poseOrientationsMatchSyntheticHands() {
        let jet = StartingPose.stickAndThrottle
        #expect(simd_length(HandMetrics.rotationVector(current: jet.stick.orientation(for: .right), neutral: rightThumbUp)) < 1e-9)
        #expect(simd_length(HandMetrics.rotationVector(current: jet.throttle.orientation(for: .left), neutral: identity)) < 1e-9)
        let drone = StartingPose.controllerGrip
        #expect(simd_length(HandMetrics.rotationVector(current: drone.throttle.orientation(for: .left), neutral: leftThumbUp)) < 1e-9)
    }

    @Test func jetCalibratesOnlyInStickAndThrottlePose() {
        let wrong = calibrate(.stickAndThrottle, left: identity, right: identity)
        #expect(wrong.result == nil)
        #expect(wrong.right == .wrongPose)
        #expect(wrong.left == .ready)

        let right = calibrate(.stickAndThrottle, left: identity, right: rightThumbUp)
        #expect(right.result != nil)
    }

    @Test func leftHandedPilotsGetTheMirroredPose() {
        // Left-handed: stick (thumb up) in the left hand, throttle flat in the right.
        let progress = calibrate(.stickAndThrottle, left: leftThumbUp, right: identity, dominant: .left)
        #expect(progress.result != nil)
    }

    @Test func verticalStickHandFliesInThePilotsFrame() {
        var interpreter = PilotInputInterpreter(calibration: HandCalibration(left: identity, right: rightThumbUp))
        var t = 0.0
        func command(_ q: Quat) -> Vec3 {
            var input = PilotInput()
            for _ in 0..<120 {
                t += 1.0 / 90
                input = interpreter.update(left: hand(.left, identity), right: hand(.right, q), time: t)
            }
            return input.rotation
        }
        // Tip the "stick" forward (fingertips down) 45°: nose down, nothing else.
        let forward = command(Quat(angle: -.pi / 4, axis: BodyAxis.right) * rightThumbUp)
        #expect(abs(forward.x - -0.5) < 0.03)
        #expect(abs(forward.y) < 0.03 && abs(forward.z) < 0.03)
        // Twist the wrist about vertical 45° to the left: yaw left.
        let twist = command(Quat(angle: .pi / 4, axis: BodyAxis.up) * rightThumbUp)
        #expect(abs(twist.y - 0.5) < 0.03)
        #expect(abs(twist.x) < 0.03 && abs(twist.z) < 0.03)
    }

    @Test func pedalWorksFromAVerticalThrottleHand() {
        var interpreter = PilotInputInterpreter(calibration: HandCalibration(left: leftThumbUp, right: rightThumbUp))
        var input = PilotInput()
        let pushed = Quat(angle: -0.4, axis: BodyAxis.right) * leftThumbUp  // fingertips down
        for i in 0..<120 { input = interpreter.update(left: hand(.left, pushed), right: hand(.right, rightThumbUp), time: Double(i) / 90) }
        #expect(input.throttle > 0.85)
    }
}
