import simd

/// Scoring: gates in order build a streak; skipping ahead still counts but breaks the streak.
public struct ScoringRules: Sendable, Equatable {
    public var gatePoints = 100
    public var streakBonus = 50
    public var outOfOrderPoints = 50
    /// Bonus for finishing, minus a per-second time penalty (floored at zero).
    public var finishBonus = 2000
    public var finishPenaltyPerSecond = 10

    public init() {}
}

/// Detects gate passes and keeps score. Feed it the craft position every frame.
public struct CourseTracker: Sendable {
    public enum GateState: Sendable, Equatable {
        case pending
        /// The gate to fly through next.
        case next
        case passed(at: Double)
    }

    public struct Event: Sendable, Equatable {
        public var gateID: Int
        public var points: Int
        public var inOrder: Bool
        public var finished: Bool
    }

    public let course: Course
    public var rules: ScoringRules
    public private(set) var score = 0
    public private(set) var streak = 0
    public private(set) var passed: [Int: Double] = [:]
    public private(set) var startTime: Double?
    public private(set) var finishTime: Double?
    private var lastPosition: Vec3?

    public init(course: Course, rules: ScoringRules = .init()) {
        self.course = course
        self.rules = rules
    }

    /// Index of the next gate in order, or nil when all are passed.
    public var nextGateID: Int? { course.gates.first { passed[$0.id] == nil }?.id }

    public func state(of gate: Gate) -> GateState {
        if let time = passed[gate.id] { return .passed(at: time) }
        return gate.id == nextGateID ? .next : .pending
    }

    public var gatesPassed: Int { passed.count }

    public var isFinished: Bool { finishTime != nil }

    /// Course time: from first movement until now, or until the last gate once finished.
    public func elapsed(at now: Double) -> Double {
        guard let start = startTime else { return 0 }
        return (finishTime ?? now) - start
    }

    /// Call once per frame. Returns the gates flown through since the last call.
    public mutating func update(position: Vec3, time: Double) -> [Event] {
        defer { lastPosition = position }
        guard let from = lastPosition, from != position else { return [] }
        if startTime == nil, simd_distance(from, position) > 0 { startTime = time }

        var events: [Event] = []
        for gate in course.gates where passed[gate.id] == nil {
            guard Self.crosses(gate, from: from, to: position) else { continue }
            let inOrder = gate.id == nextGateID
            var points: Int
            if inOrder {
                points = rules.gatePoints + rules.streakBonus * streak
                streak += 1
            } else {
                points = rules.outOfOrderPoints
                streak = 0
            }
            passed[gate.id] = time
            var finished = false
            if passed.count == course.gates.count {
                finishTime = time
                let elapsed = time - (startTime ?? time)
                points += max(0, rules.finishBonus - Int(elapsed) * rules.finishPenaltyPerSecond)
                finished = true
            }
            score += points
            events.append(Event(gateID: gate.id, points: points, inOrder: inOrder, finished: finished))
        }
        return events
    }

    /// True if the segment crosses the gate's plane in the gate's direction, inside its radius.
    static func crosses(_ gate: Gate, from a: Vec3, to b: Vec3) -> Bool {
        let da = simd_dot(a - gate.center, gate.direction)
        let db = simd_dot(b - gate.center, gate.direction)
        guard da < 0, db >= 0 else { return false }
        let t = da / (da - db)
        let hit = a + (b - a) * t
        return simd_distance(hit, gate.center) <= gate.radius
    }
}
