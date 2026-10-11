import Foundation

/// A dial scale for an instrument: maps a value to a fraction of the dial's sweep and lists
/// its tick marks. Linear for aircraft airspeed; logarithmic (after a short linear start) for
/// spacecraft, whose speeds span metres to hundreds of kilometres per second.
public struct GaugeScale: Sendable, Equatable {
    public enum Kind: Sendable, Equatable {
        /// 0…max, a major tick every `major` and a minor tick every `minor`.
        case linear(max: Double, major: Double, minor: Double)
        /// 0…`start` linearly over the first `startFraction` of the sweep, then one equal
        /// segment per decade up to `start · 10^decades`.
        case logarithmic(start: Double, decades: Int, startFraction: Double)
    }

    public struct Tick: Sendable, Equatable {
        public var fraction: Double
        public var major: Bool
        public var label: String?
    }

    public var kind: Kind
    public var unit: String
    public var title: String

    public init(kind: Kind, unit: String, title: String = "SPEED") {
        self.kind = kind
        self.unit = unit
        self.title = title
    }

    public var maxValue: Double {
        switch kind {
        case .linear(let max, _, _): max
        case .logarithmic(let start, let decades, _): start * pow(10, Double(decades))
        }
    }

    /// Position of `value` along the sweep, 0…1.
    public func fraction(for value: Double) -> Double {
        let v = max(0, value)
        switch kind {
        case .linear(let max, _, _):
            return min(1, v / max)
        case .logarithmic(let start, let decades, let s0):
            if v <= start { return s0 * v / start }
            return min(1, s0 + (1 - s0) * log10(v / start) / Double(decades))
        }
    }

    public var ticks: [Tick] {
        switch kind {
        case .linear(let max, let major, let minor):
            var ticks: [Tick] = []
            var v = 0.0
            while v <= max + 1e-9 {
                let isMajor = abs(v / major - (v / major).rounded()) < 1e-6
                ticks.append(Tick(fraction: fraction(for: v), major: isMajor, label: isMajor ? Self.format(v) : nil))
                v += minor
            }
            return ticks
        case .logarithmic(let start, let decades, _):
            var ticks = [Tick(fraction: 0, major: true, label: "0")]
            for d in 0...decades {
                let base = start * pow(10, Double(d))
                ticks.append(Tick(fraction: fraction(for: base), major: true, label: Self.format(base)))
                if d < decades {
                    for m in 2...9 { ticks.append(Tick(fraction: fraction(for: base * Double(m)), major: false, label: nil)) }
                }
            }
            return ticks
        }
    }

    /// Compact labels: 350, 1k, 10k, 100k.
    public static func format(_ v: Double) -> String {
        if v >= 1000 { return "\(Int(v / 1000))k" }
        return "\(Int(v))"
    }

    public static let spacecraft = GaugeScale(kind: .logarithmic(start: 10, decades: 4, startFraction: 0.12), unit: "m/s")
    public static let jetAirspeed = GaugeScale(kind: .linear(max: 350, major: 50, minor: 10), unit: "m/s", title: "AIRSPEED")
}
