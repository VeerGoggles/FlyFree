/// Stage 4: "the translator". Body wrench demand → per-actuator commands.
///
/// Actuators are grouped by `priority`. Each group solves a bounded weighted
/// least-squares problem  min ‖W(B·E·u − w)‖²  s.t. u ∈ range  for the residual demand
/// left by earlier groups (projected gradient descent, warm-started each frame). `E` is a
/// diagonal of per-actuator effectiveness — 1 for engines and thrusters, dynamic pressure for
/// control surfaces — so the same solver flies a spacecraft in vacuum and a jet at any speed.
public struct ControlAllocator: Sendable {
    public struct Weights: Sendable, Equatable {
        public var force: Double
        public var torque: Double
        public init(force: Double = 1, torque: Double = 1) {
            self.force = force
            self.torque = torque
        }
    }

    private struct Group: Sendable {
        var indices: [Int]
        var baseColumns: [[Double]]  // weighted unit wrenches, 6 each
        var columns: [[Double]]
        var stepSize: Double
        var effectiveness: [Double]
    }

    public let actuators: [Actuator]
    public let weights: Weights
    public var iterations = 60
    private var groups: [Group]
    private var commands: [Double]
    private var effectiveness: [Double]

    public init(actuators: [Actuator], weights: Weights = .init()) {
        self.actuators = actuators
        self.weights = weights
        self.commands = Array(repeating: 0, count: actuators.count)
        self.effectiveness = Array(repeating: 1, count: actuators.count)
        let w = [weights.force, weights.force, weights.force, weights.torque, weights.torque, weights.torque]
        let priorities = Set(actuators.map(\.priority)).sorted()
        self.groups = priorities.map { priority in
            let indices = actuators.indices.filter { actuators[$0].priority == priority }
            let columns = indices.map { i in zip(actuators[i].unitWrench.components, w).map { $0 * $1 } }
            return Group(indices: indices, baseColumns: columns, columns: columns,
                         stepSize: Self.stepSize(columns), effectiveness: indices.map { _ in 1 })
        }
    }

    /// Latest commands, index-aligned with `actuators`.
    public var currentCommands: [Double] { commands }

    /// Solves for actuator commands; returns the wrench they actually produce.
    /// - Parameter effectiveness: per-actuator scale on the unit wrench (e.g. q̄ for surfaces).
    @discardableResult
    public mutating func allocate(_ demand: Wrench, effectiveness: [Double]? = nil) -> Wrench {
        if let effectiveness { setEffectiveness(effectiveness) }
        let w = [weights.force, weights.force, weights.force, weights.torque, weights.torque, weights.torque]
        var residual = zip(demand.components, w).map { $0 * $1 }

        for group in groups {
            var u = group.indices.map { commands[$0] }
            if group.stepSize > 0 {
                for _ in 0..<iterations {
                    var r = residual.map { -$0 }
                    for (j, column) in group.columns.enumerated() where u[j] != 0 {
                        for k in 0..<6 { r[k] += column[k] * u[j] }
                    }
                    for (j, column) in group.columns.enumerated() {
                        var g = 0.0
                        for k in 0..<6 { g += column[k] * r[k] }
                        u[j] = (u[j] - group.stepSize * g).clamped(to: actuators[group.indices[j]].range)
                    }
                }
            }
            for (j, column) in group.columns.enumerated() {
                for k in 0..<6 { residual[k] -= column[k] * u[j] }
                commands[group.indices[j]] = u[j]
            }
        }
        return achievedWrench
    }

    public var achievedWrench: Wrench {
        actuators.indices.reduce(Wrench.zero) { $0 + actuators[$1].unitWrench * (commands[$1] * effectiveness[$1]) }
    }

    public mutating func reset() {
        commands = Array(repeating: 0, count: actuators.count)
    }

    private mutating func setEffectiveness(_ values: [Double]) {
        effectiveness = values
        for g in groups.indices {
            let e = groups[g].indices.map { values[$0] }
            // Re-derive the solver step only when effectiveness has changed noticeably.
            let changed = zip(e, groups[g].effectiveness).contains { abs($0 - $1) > 0.01 * max(abs($1), 1e-9) }
            guard changed else { continue }
            groups[g].effectiveness = e
            groups[g].columns = zip(groups[g].baseColumns, e).map { column, scale in column.map { $0 * scale } }
            groups[g].stepSize = Self.stepSize(groups[g].columns)
        }
    }

    private static func stepSize(_ columns: [[Double]]) -> Double {
        let lambda = largestEigenvalue(columns: columns)
        return lambda > 0 ? 1 / lambda : 0
    }

    /// Largest eigenvalue of BᵀB (= of BBᵀ, a 6×6 matrix) by power iteration.
    private static func largestEigenvalue(columns: [[Double]]) -> Double {
        var m = Array(repeating: Array(repeating: 0.0, count: 6), count: 6)
        for c in columns { for i in 0..<6 { for j in 0..<6 { m[i][j] += c[i] * c[j] } } }
        var v = Array(repeating: 1.0, count: 6)
        var lambda = 0.0
        for _ in 0..<60 {
            var next = (0..<6).map { i in (0..<6).reduce(0) { $0 + m[i][$1] * v[$1] } }
            let norm = next.reduce(0) { $0 + $1 * $1 }.squareRoot()
            guard norm > 0 else { return 0 }
            lambda = norm / v.reduce(0) { $0 + $1 * $1 }.squareRoot()
            next = next.map { $0 / norm }
            v = next
        }
        return lambda
    }
}
