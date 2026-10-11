import Testing
import simd
@testable import FlyFreeCore

/// The stick in the global frame: the same hand motion always means the same world rotation,
/// whatever the ship's attitude, with no singularities.
@Suite struct ControlFrameTests {
    let craft = CraftCatalog.genericSpacecraft
    let identity = Quat(ix: 0, iy: 0, iz: 0, r: 1)

    /// Starts the ship at `attitude`, applies a hand command, and returns the world-frame
    /// rotation axis after it has spun up.
    private func worldRotationAxis(attitude: Quat, command: Vec3, frame: ControlFrame) -> Vec3 {
        var pipeline = FlightPipeline(craft: craft, assist: .off, controlFrame: frame,
                                      initialState: RigidBodyState(orientation: attitude))
        // The reset attitude is the launch frame, not the test's starting attitude.
        pipeline.mapper.resetAttitude = identity
        for _ in 0..<45 { _ = pipeline.step(input: PilotInput(rotation: command, engaged: true), dt: 1 / 90) }
        let w = pipeline.state.orientation.act(pipeline.state.angularVelocity)
        return simd_normalize(w)
    }

    @Test(arguments: [
        Quat(angle: .pi / 2, axis: BodyAxis.up),                        // yawed 90°
        Quat(angle: .pi / 2, axis: BodyAxis.right),                     // nose straight up (Euler singularity)
        Quat(angle: .pi * 0.9, axis: simd_normalize(Vec3(1, 1, 0))),    // nearly inverted, oblique
    ])
    func globalPitchIsTheSameWorldRotationAtAnyAttitude(attitude: Quat) {
        let axis = worldRotationAxis(attitude: attitude, command: Vec3(0.8, 0, 0), frame: .global)
        #expect(simd_dot(axis, BodyAxis.right) > 0.995)
    }

    @Test func bodyFramePitchFollowsTheShip() {
        let yawed = Quat(angle: .pi / 2, axis: BodyAxis.up)
        let axis = worldRotationAxis(attitude: yawed, command: Vec3(0.8, 0, 0), frame: .body)
        // Body X of a ship yawed 90° left points along world +Z.
        #expect(simd_dot(axis, yawed.act(BodyAxis.right)) > 0.995)
    }

    @Test func combinedCommandsAreAFixedWorldAxisThroughLargeRotations() {
        // Hold pitch + yaw for long enough to tumble well past any Euler singularity.
        var pipeline = FlightPipeline(craft: craft, assist: .off, controlFrame: .global)
        let command = PilotInput(rotation: Vec3(0.7, 0.7, 0), engaged: true)
        var axes: [Vec3] = []
        for i in 0..<(90 * 12) {
            _ = pipeline.step(input: command, dt: 1 / 90)
            if i > 270, i % 90 == 0 {
                axes.append(simd_normalize(pipeline.state.orientation.act(pipeline.state.angularVelocity)))
            }
        }
        // The world rotation axis never wanders while the ship rotates through > 360°.
        for axis in axes { #expect(simd_dot(axis, axes[0]) > 0.995) }
        #expect(abs(pipeline.state.orientation.length - 1) < 1e-9)
        #expect(!pipeline.state.angularVelocity.x.isNaN)
    }

    @Test func resetReturnsHomeFromAnyAttitude() {
        var pipeline = FlightPipeline(craft: craft, assist: .off, controlFrame: .global,
                                      initialState: RigidBodyState(orientation: Quat(angle: 3.0, axis: simd_normalize(Vec3(0.3, -1, 0.5)))))
        pipeline.mapper.resetAttitude = identity
        for _ in 0..<(90 * 15) { _ = pipeline.step(input: PilotInput(stickClosed: true), dt: 1 / 90) }
        #expect(simd_length(HandMetrics.rotationVector(current: pipeline.state.orientation, neutral: identity)) < 0.01)
    }

    @Test func pedalTwistMatchesHandTilt() {
        let neutral = Quat(angle: 0.2, axis: BodyAxis.up)  // calibrated with the hand turned a little
        for tilt in [-0.4, -0.1, 0.25, 0.45] {
            let current = neutral * Quat(angle: tilt, axis: BodyAxis.right)
            #expect(abs(HandMetrics.twistAngle(current: current, neutral: neutral, about: BodyAxis.right) - tilt) < 1e-9)
        }
    }
}
