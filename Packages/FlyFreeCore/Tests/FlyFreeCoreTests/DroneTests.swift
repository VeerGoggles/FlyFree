import Testing
import simd
@testable import FlyFreeCore

/// The drone flies with the jet's controls through a position/velocity flight computer.
@Suite struct DroneTests {
    let drone = CraftCatalog.drone
    let dt = 1.0 / 90

    private func run(_ p: inout FlightPipeline, _ input: PilotInput, seconds: Double) -> FlightTelemetry {
        var t = p.step(input: input, dt: dt)
        for _ in 1..<Int(seconds * 90) { t = p.step(input: input, dt: dt) }
        return t
    }

    /// Takes off from the pad with the jet's gesture and levels off.
    private func airborne() -> FlightPipeline {
        var p = FlightPipeline(craft: drone)
        _ = run(&p, PilotInput(rotation: Vec3(1, 0, 0), engaged: true), seconds: 5)
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        return p
    }

    private func bank(_ q: Quat) -> Double { IntentMapper.bank(q) }

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

    @Test func startsInTheJetsPose() {
        #expect(drone.startingPose == .stickAndThrottle)
    }

    @Test func sitsOnThePadAndThePedalAloneDoesNotDragItAlong() {
        var p = FlightPipeline(craft: drone)
        let t = run(&p, PilotInput(throttle: 1, engaged: true), seconds: 3)
        #expect(t.wheelsOnGround == 4)
        #expect(simd_length(t.state.velocity) < 0.05)
    }

    @Test func takeOffGestureIsTheJetsPullBackPlusPedal() {
        var p = FlightPipeline(craft: drone)
        // Pull back: lifts straight off the pad and climbs.
        var t = run(&p, PilotInput(rotation: Vec3(1, 0, 0), engaged: true), seconds: 3)
        #expect(!t.onGround)
        #expect(t.state.velocity.y > 4)
        // Add pedal: speeds up forward while still climbing.
        t = run(&p, PilotInput(rotation: Vec3(0.6, 0, 0), throttle: 1, engaged: true), seconds: 4)
        #expect(-t.state.velocity.z > 8)
        #expect(t.state.velocity.y > 0.5)
    }

    @Test func pedalSetsSpeedAndReleasedStickHoldsAltitude() {
        var p = airborne()
        _ = run(&p, PilotInput(throttle: 1, engaged: true), seconds: 3)      // full pedal for 3 s
        let y = p.state.position.y
        let t = run(&p, PilotInput(engaged: true), seconds: 6)               // level pedal: hold
        let speed = simd_length(Vec3(t.state.velocity.x, 0, t.state.velocity.z))
        #expect(abs(speed - 3 * drone.profile.linearAcceleration) < 1)
        #expect(abs(t.state.position.y - y) < 1)
    }

    @Test func rollMakesACoordinatedBankedTurnAndReleaseHoldsHeading() {
        var p = airborne()
        _ = run(&p, PilotInput(throttle: 1, engaged: true), seconds: 3)
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        let h0 = IntentMapper.heading(p.state.orientation), y0 = p.state.position.y
        var t = run(&p, PilotInput(rotation: Vec3(0, 0, -1), engaged: true), seconds: 3)  // roll right
        #expect(IntentMapper.heading(t.state.orientation) < h0 - 0.5)   // turned right
        #expect(bank(t.state.orientation) > 0.2)                        // banked into the turn
        #expect(abs(t.state.position.y - y0) < 1.5)
        // Velocity follows the heading (no skidding): the craft flies where it points.
        let v = simd_normalize(Vec3(t.state.velocity.x, 0, t.state.velocity.z))
        var f = t.state.orientation.act(BodyAxis.forward); f.y = 0
        #expect(simd_dot(v, simd_normalize(f)) > 0.95)
        // Release: wings level, heading held.
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        let h1 = IntentMapper.heading(p.state.orientation)
        t = run(&p, PilotInput(engaged: true), seconds: 3)
        #expect(abs(IntentMapper.heading(t.state.orientation) - h1) < 0.05)
        #expect(abs(bank(t.state.orientation)) < 0.05)
    }

    @Test func turnsInPlaceAtAHover() {
        var p = airborne()
        let start = p.state.position
        let t = run(&p, PilotInput(rotation: Vec3(0, 0, 1), engaged: true), seconds: 3)
        #expect(IntentMapper.heading(t.state.orientation) > 1.0)
        #expect(simd_distance(t.state.position, start) < 1)
    }

    @Test func holdsPositionAgainstADisturbance() {
        var p = airborne()
        _ = run(&p, PilotInput(engaged: true), seconds: 2)
        let hold = p.state.position
        p.state.velocity += Vec3(3, -1, 2)   // a gust
        let t = run(&p, PilotInput(engaged: true), seconds: 8)
        #expect(simd_distance(t.state.position, hold) < 0.5)
    }

    @Test func closedThrottleHandBrakesToAHover() {
        var p = airborne()
        _ = run(&p, PilotInput(throttle: 1, engaged: true), seconds: 4)
        let t = run(&p, PilotInput(engaged: true, throttleClosed: true), seconds: 6)
        #expect(simd_length(t.state.velocity) < 0.3)
    }

    @Test func pushingForwardDescendsAndLands() {
        var p = airborne()
        let t = run(&p, PilotInput(rotation: Vec3(-1, 0, 0), engaged: true), seconds: 10)
        #expect(t.wheelsOnGround == 4)
        #expect(abs(t.state.velocity.y) < 0.1)
    }
}
