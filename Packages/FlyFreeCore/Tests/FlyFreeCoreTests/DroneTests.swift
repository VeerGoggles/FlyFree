import Testing
import simd
@testable import FlyFreeCore

@Suite struct DroneTests {
    let drone = CraftCatalog.drone
    let dt = 1.0 / 90

    private func run(_ p: inout FlightPipeline, _ input: PilotInput, seconds: Double) -> FlightTelemetry {
        var t = p.step(input: input, dt: dt)
        for _ in 1..<Int(seconds * 90) { t = p.step(input: input, dt: dt) }
        return t
    }

    private func hovering(at altitude: Double = 30) -> FlightPipeline {
        var p = FlightPipeline(craft: drone, initialState: RigidBodyState(position: Vec3(0, altitude, 0)))
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        return p
    }

    @Test func rotorsMixLikeAQuadcopter() {
        var allocator = ControlAllocator(actuators: drone.actuators, weights: drone.allocationWeights)
        let hover = Wrench(force: Vec3(0, drone.mass.mass * 9.81, 0))
        allocator.allocate(hover)
        for c in allocator.currentCommands { #expect(abs(c - 0.44) < 0.01) }
        // A yaw demand speeds up one diagonal pair and slows the other, keeping total lift.
        let achieved = allocator.allocate(hover + Wrench(torque: Vec3(0, 200, 0)))
        #expect(abs(achieved.force.y - hover.force.y) < 20)
        #expect(achieved.torque.y > 190)
        let c = allocator.currentCommands
        #expect(abs(c[0] - c[2]) < 0.01 && abs(c[1] - c[3]) < 0.01 && abs(c[0] - c[1]) > 0.1)
    }

    @Test func sitsOnTheHelipad() {
        var p = FlightPipeline(craft: drone)
        let t = run(&p, .neutral, seconds: 3)
        #expect(t.wheelsOnGround == 4)
        #expect(simd_length(t.state.velocity) < 0.02)
    }

    @Test func pedalClimbsAndLevelPedalHoldsAltitude() {
        var p = FlightPipeline(craft: drone)
        var t = run(&p, PilotInput(throttle: 0.9), seconds: 6)
        #expect(!t.onGround)
        #expect(t.state.velocity.y > 4)
        _ = run(&p, PilotInput(engaged: true), seconds: 4)
        let y = p.state.position.y
        t = run(&p, PilotInput(engaged: true), seconds: 5)
        #expect(abs(t.state.position.y - y) < 0.5)
        #expect(simd_length(t.state.velocity) < 0.2)
    }

    @Test func tiltFliesForwardAtConstantAltitudeAndReleaseHovers() {
        var p = hovering()
        var t = run(&p, PilotInput(rotation: Vec3(-0.9, 0, 0), engaged: true), seconds: 8)
        let expected = drone.profile.rotationShaping.apply(0.9) * drone.multirotor!.maxHorizontalSpeed
        #expect(abs(-t.state.velocity.z - expected) < 0.5)   // forward = −Z at launch heading
        #expect(abs(t.state.position.y - 30) < 1.5)
        // Let go: brakes to a hover within ~4 s.
        t = run(&p, PilotInput(engaged: true), seconds: 4)
        #expect(simd_length(t.state.velocity) < 0.3)
        #expect(acos(min(1, t.state.orientation.act(BodyAxis.up).y)) < 0.05)
    }

    @Test func yawTurnsAndHoldsHeadingWithoutOvershoot() {
        var p = hovering()
        _ = run(&p, PilotInput(rotation: Vec3(0, 0.8, 0), engaged: true), seconds: 2)
        let released = IntentMapper.heading(p.state.orientation)
        #expect(released > 0.5)
        let t = run(&p, PilotInput(engaged: true), seconds: 4)
        let settled = IntentMapper.heading(t.state.orientation)
        #expect(abs(settled - released) < 0.15)
        #expect(abs(t.state.angularVelocity.y) < 0.02)
    }

    @Test func closedHandsHoverAndTurnBackToLaunchHeading() {
        var p = hovering()
        _ = run(&p, PilotInput(rotation: Vec3(-0.9, 0.7, 0), engaged: true), seconds: 4)
        var t = run(&p, PilotInput(engaged: true, throttleClosed: true), seconds: 5)
        #expect(simd_length(t.state.velocity) < 0.3)
        t = run(&p, PilotInput(stickClosed: true), seconds: 6)
        #expect(abs(IntentMapper.heading(t.state.orientation)) < 0.03)
    }

    @Test func descendsAndLands() {
        var p = hovering(at: 12)
        let t = run(&p, PilotInput(throttle: -0.9), seconds: 8)
        #expect(t.wheelsOnGround == 4)
        #expect(abs(t.state.velocity.y) < 0.1)
    }
}
