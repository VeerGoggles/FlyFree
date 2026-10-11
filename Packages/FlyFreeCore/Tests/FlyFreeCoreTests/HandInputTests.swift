import Testing
import simd
@testable import FlyFreeCore

@Suite struct HandInputTests {
    @Test func straightFingerHasNoFlexion() {
        let hand = SyntheticHand.make(chirality: .right)
        #expect(HandMetrics.flexion(hand, .index) < 0.01)
    }

    @Test func fistHasLargeFlexion() {
        let hand = SyntheticHand.make(chirality: .right, curl: [.index: 1, .middle: 1])
        #expect(HandMetrics.flexion(hand, .middle) > 4.0)
    }

    @Test(arguments: Chirality.allCases)
    func neutralHandOrientationIsIdentity(chirality: Chirality) throws {
        let hand = SyntheticHand.make(chirality: chirality)
        let q = try #require(HandMetrics.orientation(of: hand))
        let angles = HandMetrics.relativeAngles(current: q, neutral: Quat(ix: 0, iy: 0, iz: 0, r: 1))
        #expect(simd_length(angles) < 1e-6)
    }

    @Test(arguments: Chirality.allCases)
    func relativeAnglesRecoverHandRotation(chirality: Chirality) throws {
        let rotation = Quat.fromYawPitchRoll(yaw: 0.2, pitch: -0.3, roll: 0.25)
        let hand = SyntheticHand.make(chirality: chirality, orientation: rotation, position: Vec3(0.2, 1.1, -0.3))
        let q = try #require(HandMetrics.orientation(of: hand))
        let angles = HandMetrics.relativeAngles(current: q, neutral: Quat(ix: 0, iy: 0, iz: 0, r: 1))
        #expect(abs(angles.x - -0.3) < 1e-6)
        #expect(abs(angles.y - 0.2) < 1e-6)
        #expect(abs(angles.z - 0.25) < 1e-6)
    }

    @Test func gripEngagesAndCapturesNeutral() {
        var interpreter = PilotInputInterpreter()
        let tilted = Quat.fromYawPitchRoll(yaw: 0.4, pitch: 0, roll: 0)
        let fist: [Finger: Double] = [.index: 0.9, .middle: 0.9, .ring: 0.9, .little: 0.9]

        // Open hand: not engaged.
        var input = interpreter.update(left: nil, right: SyntheticHand.make(chirality: .right, orientation: tilted), time: 0)
        #expect(!input.engaged)

        // Grip while tilted: engages, and that pose becomes neutral → no rotation command.
        input = interpreter.update(left: nil, right: SyntheticHand.make(chirality: .right, orientation: tilted, curl: fist), time: 0.01)
        #expect(input.engaged)
        #expect(simd_length(input.rotation) < 1e-6)

        // Pitch the hand up from there → positive pitch command.
        let pitched = tilted * Quat(angle: 0.3, axis: BodyAxis.right)
        for i in 2..<60 {
            input = interpreter.update(left: nil, right: SyntheticHand.make(chirality: .right, orientation: pitched, curl: fist), time: Double(i) * 0.011)
        }
        #expect(input.rotation.x > 0.4)
        #expect(abs(input.rotation.y) < 0.05)
    }

    @Test func offHandCurlDrivesThrottle() {
        var config = InterpreterConfiguration()
        config.dominantHand = .left
        var interpreter = PilotInputInterpreter(configuration: config)
        let curls: [Finger: Double] = [.index: 1, .middle: 1, .ring: 1, .little: 1]
        var input = PilotInput()
        for i in 0..<120 {
            input = interpreter.update(left: nil, right: SyntheticHand.make(chirality: .right, curl: curls), time: Double(i) / 90)
        }
        #expect(input.throttle > 0.95)

        // Throttle hand lost → held briefly, then zeroed.
        input = interpreter.update(left: nil, right: nil, time: 1.5)
        #expect(input.throttle > 0.95)
        input = interpreter.update(left: nil, right: nil, time: 3.0)
        #expect(input.throttle == 0)
    }
}
