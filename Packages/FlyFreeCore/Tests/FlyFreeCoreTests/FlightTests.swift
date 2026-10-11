import Testing
import simd
@testable import FlyFreeCore

@Suite struct FlightTests {
    let craft = CraftCatalog.genericSpacecraft

    @Test func pureTorqueDemandUsesRCSWithoutNetForce() {
        var allocator = ControlAllocator(actuators: craft.actuators, weights: craft.allocationWeights)
        let achieved = allocator.allocate(Wrench(torque: Vec3(20_000, 0, 0)))
        #expect(abs(achieved.torque.x - 20_000) < 200)
        #expect(simd_length(achieved.force) < 200)
        for (actuator, command) in zip(craft.actuators, allocator.currentCommands) where actuator.kind == .mainEngine {
            #expect(command < 1e-6)
        }
    }

    @Test func forwardDemandFiresMainEngines() {
        var allocator = ControlAllocator(actuators: craft.actuators, weights: craft.allocationWeights)
        let achieved = allocator.allocate(Wrench(force: BodyAxis.forward * 1e6))
        #expect(abs(achieved.force.z - -craft.maxForwardThrust) < 1)
        #expect(simd_length(achieved.torque) < 1)
    }

    @Test func coastsAtConstantVelocityInDeepSpace() {
        var state = RigidBodyState(velocity: Vec3(0, 0, -10))
        for _ in 0..<240 {
            RigidBodyIntegrator.step(&state, mass: craft.mass, bodyWrench: .zero, external: .none, dt: 1 / 240)
        }
        #expect(abs(state.position.z - -10) < 1e-9)
    }

    @Test func pitchCommandPitchesNoseUpThenHolds() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        let pitchUp = PilotInput(rotation: Vec3(1, 0, 0), engaged: true)
        for _ in 0..<(90 * 3) { _ = pipeline.step(input: pitchUp, dt: 1 / 90) }
        let nose = pipeline.state.orientation.act(BodyAxis.forward)
        #expect(nose.y > 0.5)

        // Release the stick: rotation stops (attitude hold), limited by RCS authority.
        for _ in 0..<(90 * 5) { _ = pipeline.step(input: .neutral, dt: 1 / 90) }
        #expect(simd_length(pipeline.state.angularVelocity) < 0.01)
    }

    @Test func fullThrottleKeepsAcceleratingTowardOneHundredTwentyKmPerSecond() {
        #expect(craft.profile.maxForwardSpeed == 120_000)
        var pipeline = FlightPipeline(craft: craft, assist: .velocityHold)
        var telemetry = pipeline.step(input: PilotInput(throttle: 1), dt: 1 / 90)
        for _ in 0..<(90 * 30) { telemetry = pipeline.step(input: PilotInput(throttle: 1), dt: 1 / 90) }
        // Main engines saturate at 120 kN on 12 t → 10 m/s² for the whole climb.
        #expect(abs(telemetry.speed - 300) < 5)
    }

    @Test func velocityHoldReachesAndThenStops() {
        // Velocity-loop behaviour, on a slow-capped variant so the target is reachable quickly.
        var slow = craft
        slow.profile.maxForwardSpeed = 120
        var pipeline = FlightPipeline(craft: slow, assist: .velocityHold)
        let half = PilotInput(throttle: 0.3)
        var telemetry = pipeline.step(input: half, dt: 1 / 90)
        for _ in 0..<(90 * 20) { telemetry = pipeline.step(input: half, dt: 1 / 90) }
        let target = -slow.profile.throttleShaping.apply(0.3) * slow.profile.maxForwardSpeed
        #expect(abs(telemetry.state.velocity.z - target) < 1)

        // Throttle to zero: retro thrusters bring the craft to rest.
        for _ in 0..<(90 * 60) { telemetry = pipeline.step(input: .neutral, dt: 1 / 90) }
        #expect(telemetry.speed < 0.5)
    }

    @Test func releasedStickHoldsAttitudeAgainstADisturbance() {
        var pipeline = FlightPipeline(craft: craft, assist: .off)
        _ = pipeline.step(input: .neutral, dt: 1 / 90)
        pipeline.state.angularVelocity = Vec3(0, 0.2, 0)  // a knock
        for _ in 0..<(90 * 8) { _ = pipeline.step(input: .neutral, dt: 1 / 90) }
        let drift = HandMetrics.rotationVector(current: pipeline.state.orientation, neutral: Quat(ix: 0, iy: 0, iz: 0, r: 1))
        #expect(simd_length(drift) < 0.01)
    }

    @Test func flatHandsDriveTheWholePipeline() {
        var pipeline = FlightPipeline(craft: craft, calibration: .identity)
        let left = SyntheticHand.make(chirality: .left, position: Vec3(-0.2, 1, -0.35))
        // Forearm rotated thumb-up → roll right (negative Z).
        let right = SyntheticHand.make(chirality: .right, orientation: Quat(angle: -0.5, axis: Vec3(0, 0, 1)), position: Vec3(0.2, 1, -0.35))
        var telemetry = pipeline.step(left: left, right: right, dt: 1 / 90)
        for _ in 0..<90 { telemetry = pipeline.step(left: left, right: right, dt: 1 / 90) }
        #expect(telemetry.input.engaged)
        #expect(telemetry.state.angularVelocity.z < -0.3)
    }
}
