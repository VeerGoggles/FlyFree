import Testing
import simd
@testable import FlyFreeCore

@Suite struct JetTests {
    let jet = CraftCatalog.jet
    let dt = 1.0 / 90

    private func run(_ p: inout FlightPipeline, _ input: PilotInput, seconds: Double) -> FlightTelemetry {
        var t = p.step(input: input, dt: dt)
        for _ in 1..<Int(seconds * 90) { t = p.step(input: input, dt: dt) }
        return t
    }

    private func airborne(speed: Double = 150, altitude: Double = 1000) -> FlightPipeline {
        FlightPipeline(craft: jet, initialState: RigidBodyState(
            position: Vec3(0, altitude, 0), velocity: Vec3(0, 0, -speed),
            orientation: Quat(angle: 0.04, axis: BodyAxis.right)))
    }

    private func heading(_ q: Quat) -> Double {
        let f = q.act(BodyAxis.forward)
        return atan2(-f.x, -f.z)
    }

    @Test func restsOnTheRunway() {
        var p = FlightPipeline(craft: jet)
        let t = run(&p, .neutral, seconds: 3)
        #expect(t.wheelsOnGround == 3)
        #expect(abs(t.state.velocity.y) < 0.01)
        #expect(abs(t.state.position.y - 1.45) < 0.15)
    }

    @Test func surfacesDoNothingWithoutAirflow() {
        var p = FlightPipeline(craft: jet)
        let t = run(&p, PilotInput(rotation: Vec3(1, 1, 1), engaged: true), seconds: 2)
        #expect(simd_length(t.state.angularVelocity) < 0.01)
    }

    @Test func takesOffAndClimbs() {
        var p = FlightPipeline(craft: jet)
        _ = run(&p, PilotInput(throttle: 1), seconds: 16)            // take-off roll
        _ = run(&p, PilotInput(rotation: Vec3(0.6, 0, 0), engaged: true), seconds: 3)  // rotate
        let t = run(&p, PilotInput(engaged: true), seconds: 10)       // release: hold the climb
        #expect(!t.onGround)
        #expect(t.state.position.y > 80)
        #expect(t.state.velocity.y > 5)
    }

    @Test func closedStickLevelsTheWingsAndHoldsAltitude() {
        var p = airborne()
        _ = run(&p, PilotInput(rotation: Vec3(0.4, 0, -0.6), engaged: true), seconds: 1.5)  // pitch up and bank
        _ = run(&p, PilotInput(stickClosed: true), seconds: 10)
        let before = p.state.position.y
        let t = run(&p, PilotInput(stickClosed: true), seconds: 5)
        #expect(abs(UnitTestBank.of(t.state.orientation)) < 0.02)
        #expect(abs(t.state.position.y - before) < 8)
    }

    @Test func releasedBankMakesACoordinatedLevelTurn() {
        var p = airborne()
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        _ = run(&p, PilotInput(rotation: Vec3(0, 0, -0.35), engaged: true), seconds: 0.9)  // roll right
        _ = run(&p, PilotInput(engaged: true), seconds: 3)
        let bank = UnitTestBank.of(p.state.orientation)
        #expect(bank > 0.3 && bank < 0.7)
        let h0 = heading(p.state.orientation), y0 = p.state.position.y
        let t = run(&p, PilotInput(engaged: true), seconds: 5)
        // Turn rate matches g·tanφ/V; sideslip small; altitude held; bank held.
        let rate = -(heading(t.state.orientation) - h0) / 5
        let expected = 9.81 * tan(bank) / (t.air?.airspeed ?? 1)
        #expect(abs(rate - expected) / expected < 0.1)
        #expect(abs(t.air?.beta ?? 1) < 0.035)
        #expect(abs(t.state.position.y - y0) < 15)
        #expect(abs(UnitTestBank.of(t.state.orientation) - bank) < 0.05)
    }

    @Test func alphaProtectionPreventsTheStall() {
        var p = airborne(speed: 110)
        var maxAlpha = 0.0
        for _ in 0..<(90 * 8) {
            let t = p.step(input: PilotInput(rotation: Vec3(1, 0, 0), throttle: -1, engaged: true), dt: dt)
            maxAlpha = max(maxAlpha, t.air?.alpha ?? 0)
        }
        #expect(maxAlpha < jet.aircraft!.alphaLimit + 0.015)
        #expect(maxAlpha < jet.aerodynamics!.stallAlpha)
    }

    @Test func autothrottleHoldsAndChangesAirspeed() {
        var p = airborne(speed: 150)
        var t = run(&p, PilotInput(stickClosed: true), seconds: 10)
        #expect(abs((t.air?.airspeed ?? 0) - 150) < 3)
        t = run(&p, PilotInput(throttle: 1, stickClosed: true), seconds: 5)
        #expect((t.air?.airspeed ?? 0) > 170)
    }

    @Test func brakesStopOnTheRunway() {
        var p = FlightPipeline(craft: jet, initialState: RigidBodyState(position: Vec3(0, 1.45, 0), velocity: Vec3(0, 0, -50)))
        var t = p.step(input: PilotInput(throttleClosed: true), dt: dt)
        var seconds = 0.0
        while t.speed > 0.3, seconds < 30 {
            t = p.step(input: PilotInput(throttleClosed: true), dt: dt)
            seconds += dt
        }
        #expect(seconds < 15)
        #expect(abs(t.state.position.x) < 3)
        #expect(t.wheelsOnGround == 3)
    }
}

enum UnitTestBank {
    static func of(_ q: Quat) -> Double { IntentMapper.bank(q) }
}
