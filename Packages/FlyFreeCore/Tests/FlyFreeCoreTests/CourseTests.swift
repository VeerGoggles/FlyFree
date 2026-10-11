import Testing
import simd
@testable import FlyFreeCore

@Suite struct CourseTests {
    let course = Course(name: "Test", gates: [
        Gate(id: 0, center: Vec3(0, 0, -100), radius: 10),
        Gate(id: 1, center: Vec3(0, 0, -200), radius: 10),
        Gate(id: 2, center: Vec3(0, 0, -300), radius: 10),
    ])

    private func fly(_ tracker: inout CourseTracker, through points: [Vec3]) -> [CourseTracker.Event] {
        var events: [CourseTracker.Event] = []
        for (i, p) in points.enumerated() { events += tracker.update(position: p, time: Double(i)) }
        return events
    }

    @Test func passingInOrderBuildsStreakAndFinishes() {
        var tracker = CourseTracker(course: course)
        let events = fly(&tracker, through: stride(from: 0.0, through: -320, by: -20).map { Vec3(1, 2, $0) })
        #expect(events.map(\.gateID) == [0, 1, 2])
        #expect(events.map(\.inOrder) == [true, true, true])
        #expect(events.last?.finished == true)
        #expect(tracker.nextGateID == nil)
        // 100 + 150 + 200 + finish bonus (2000 − 14 s × 10: clock starts on first movement at t=1, last gate at t=15).
        #expect(tracker.score == 450 + 2000 - 140)
    }

    @Test func missingTheRingOrGoingBackwardsDoesNotCount() {
        var tracker = CourseTracker(course: course)
        _ = fly(&tracker, through: [Vec3(20, 0, -90), Vec3(20, 0, -110)])  // outside radius
        #expect(tracker.gatesPassed == 0)
        _ = fly(&tracker, through: [Vec3(0, 0, -110), Vec3(0, 0, -90)])    // wrong way
        #expect(tracker.gatesPassed == 0)
        #expect(tracker.state(of: course.gates[0]) == .next)
    }

    @Test func skippingAheadBreaksTheStreak() {
        var tracker = CourseTracker(course: course)
        _ = fly(&tracker, through: [Vec3(0, 0, -150), Vec3(0, 0, -210)])
        #expect(tracker.score == 50)
        #expect(tracker.streak == 0)
        #expect(tracker.nextGateID == 0)
    }
}
