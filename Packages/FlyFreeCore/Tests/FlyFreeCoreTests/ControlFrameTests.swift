import Testing
import simd
@testable import FlyFreeCore

/// The stick commands *deltas* about the ship's current axes: the same gesture does the same
/// thing to the ship at any attitude, with no gimbal lock.
@Suite struct ControlFrameTests {
    let craft = CraftCatalog.genericSpacecraft

    /// Starts the ship at `attitude`, applies a hand command, and returns the rotation axis
    /// in the ship's own frame after it has spun up.
    private func bodyRotationAxis(attitude: Quat, command: Vec3) -> Vec3 {
        var pipeline = FlightPipeline(craft: craft, assist: .off, initialState: RigidBodyState(orientation: attitude))
        for _ in 0..<45 { _ = pipeline.step(input: PilotInput(rotation: command, engaged: true), dt: 1 / 90) }
        return simd_normalize(pipeline.state.angularVelocity)
    }

    @Test(arguments: [
        Quat(ix: 0, iy: 0, iz: 0, r: 1),
        Quat(angle: .pi / 2, axis: BodyAxis.up),                        // yawed 90°
        Quat(angle: .pi / 2, axis: BodyAxis.right),                     // nose straight up (Euler singularity)
        Quat(angle: .pi * 0.9, axis: simd_normalize(Vec3(1, 1, 0))),    // nearly inverted, oblique
    ])
    func yawGestureYawsTheShipTheSameWayAtAnyAttitude(attitude: Quat) {
        #expect(simd_dot(bodyRotationAxis(attitude: attitude, command: Vec3(0, 0.8, 0)), BodyAxis.up) > 0.999)
        #expect(simd_dot(bodyRotationAxis(attitude: attitude, command: Vec3(0.8, 0, 0)), BodyAxis.right) > 0.999)
    }

    @Test func combinedCommandKeepsAFixedBodyAxisThroughLargeRotations() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        let command = PilotInput(rotation: Vec3(0.7, 0.7, 0.4), engaged: true)
        var axes: [Vec3] = []
        for i in 0..<(90 * 12) {
            _ = pipeline.step(input: command, dt: 1 / 90)
            if i > 90, i % 90 == 0 { axes.append(simd_normalize(pipeline.state.angularVelocity)) }
        }
        for axis in axes { #expect(simd_dot(axis, axes[0]) > 0.995) }
        #expect(abs(pipeline.state.orientation.length - 1) < 1e-9)
        #expect(!pipeline.state.angularVelocity.x.isNaN)
    }

    @Test func resetReturnsHomeFromAnyAttitude() {
        let identity = Quat(ix: 0, iy: 0, iz: 0, r: 1)
        var pipeline = FlightPipeline(craft: craft, assist: .off,
                                      initialState: RigidBodyState(orientation: Quat(angle: 3.0, axis: simd_normalize(Vec3(0.3, -1, 0.5)))))
        pipeline.mapper.resetAttitude = identity
        for _ in 0..<(90 * 10) { _ = pipeline.step(input: PilotInput(stickClosed: true), dt: 1 / 90) }
        #expect(simd_length(HandMetrics.rotationVector(current: pipeline.state.orientation, neutral: identity)) < 0.01)
    }

    @Test func pedalTwistMatchesHandTilt() {
        let neutral = Quat(angle: 0.2, axis: BodyAxis.up)
        for tilt in [-0.4, -0.1, 0.25, 0.45] {
            let current = neutral * Quat(angle: tilt, axis: BodyAxis.right)
            #expect(abs(HandMetrics.twistAngle(current: current, neutral: neutral, about: BodyAxis.right) - tilt) < 1e-9)
        }
    }
}
