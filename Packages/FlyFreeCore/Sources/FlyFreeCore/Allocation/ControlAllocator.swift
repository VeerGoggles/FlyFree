/// Stage 4: "the translator". Body wrench demand → per-actuator commands in 0...1.
///
/// Actuators are grouped by `priority`. Each group solves a bounded weighted
/// least-squares problem  min ‖W(B·u − w)‖²  s.t. 0 ≤ u ≤ 1  for the residual demand
/// left by earlier groups (projected gradient descent, warm-started each frame).
/// This keeps big main engines and small RCS thrusters from fighting in one badly
/// scaled solve, and works for any thruster layout without hand-written mixing tables.
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
        var columns: [[Double]]  // weighted unit wrenches, 6 each
        var stepSize: Double
    }

    public let actuators: [Actuator]
    public let weights: Weights
    public var iterations = 60
    private let groups: [Group]
    private var commands: [Double]

    public init(actuators: [Actuator], weights: Weights = .init()) {
        self.actuators = actuators
        self.weights = weights
        self.commands = Array(repeating: 0, count: actuators.count)
        let w = [weights.force, weights.force, weights.force, weights.torque, weights.torque, weights.torque]
        let priorities = Set(actuators.map(\.priority)).sorted()
        self.groups = priorities.map { priority in
            let indices = actuators.indices.filter { actuators[$0].priority == priority }
            let columns = indices.map { i in zip(actuators[i].unitWrench.components, w).map { $0 * $1 } }
            let lambda = Self.largestEigenvalue(columns: columns)
            return Group(indices: indices, columns: columns, stepSize: lambda > 0 ? 1 / lambda : 0)
        }
    }

    /// Latest commands, index-aligned with `actuators`.
    public var currentCommands: [Double] { commands }

    /// Solves for actuator commands; returns the wrench they actually produce.
    @discardableResult
    public mutating func allocate(_ demand: Wrench) -> Wrench {
        let w = [weights.force, weights.force, weights.force, weights.torque, weights.torque, weights.torque]
        var residual = zip(demand.components, w).map { $0 * $1 }

        for group in groups {
            var u = group.indices.map { commands[$0] }
            for _ in 0..<iterations {
                // r = B u − w
                var r = residual.map { -$0 }
                for (j, column) in group.columns.enumerated() where u[j] != 0 {
                    for k in 0..<6 { r[k] += column[k] * u[j] }
                }
                // u ← clamp(u − η Bᵀ r)
                for (j, column) in group.columns.enumerated() {
                    var g = 0.0
                    for k in 0..<6 { g += column[k] * r[k] }
                    u[j] = (u[j] - group.stepSize * g).clamped(to: 0...1)
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
        zip(actuators, commands).reduce(Wrench.zero) { $0 + $1.0.unitWrench * $1.1 }
    }

    public mutating func reset() {
        commands = Array(repeating: 0, count: actuators.count)
    }

    /// Largest eigenvalue of BᵀB (= of BBᵀ, a 6×6 matrix) by power iteration.
    private static func largestEigenvalue(columns: [[Double]]) -> Double {
        var m = Array(repeating: Array(repeating: 0.0, count: 6), count: 6)
        for c in columns { for i in 0..<6 { for j in 0..<6 { m[i][j] += c[i] * c[j] } } }
        var v = Array(repeating: 1.0, count: 6)
        var lambda = 0.0
        for _ in 0..<100 {
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
