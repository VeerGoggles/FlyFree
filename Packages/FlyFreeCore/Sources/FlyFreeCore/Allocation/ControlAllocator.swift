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
                // Small groups (e.g. 4 rotors, whose yaw torque is tiny next to lift) are badly
                // conditioned for gradient descent but cheap to solve exactly: start from the
                // unconstrained least-squares solution, clamped; the projected gradient then only
                // has to resolve any limits that bind.
                if group.indices.count <= 6, let exact = Self.leastSquares(group.columns, residual) {
                    u = zip(exact, group.indices).map { $0.clamped(to: actuators[$1].range) }
                }
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

    /// Solves (BᵀB + εI) u = Bᵀw by Gaussian elimination with partial pivoting (n ≤ 6).
    private static func leastSquares(_ columns: [[Double]], _ w: [Double]) -> [Double]? {
        let n = columns.count
        var a = [[Double]](repeating: [Double](repeating: 0, count: n + 1), count: n)
        var trace = 0.0
        for i in 0..<n {
            for j in 0..<n { a[i][j] = (0..<6).reduce(0) { $0 + columns[i][$1] * columns[j][$1] } }
            a[i][n] = (0..<6).reduce(0) { $0 + columns[i][$1] * w[$1] }
            trace += a[i][i]
        }
        guard trace > 0 else { return nil }
        for i in 0..<n { a[i][i] += 1e-9 * trace }
        for col in 0..<n {
            guard let pivot = (col..<n).max(by: { abs(a[$0][col]) < abs(a[$1][col]) }), abs(a[pivot][col]) > 1e-12 else { return nil }
            a.swapAt(col, pivot)
            for row in 0..<n where row != col {
                let f = a[row][col] / a[col][col]
                if f != 0 { for k in col...n { a[row][k] -= f * a[col][k] } }
            }
        }
        return (0..<n).map { a[$0][n] / a[$0][$0] }
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
