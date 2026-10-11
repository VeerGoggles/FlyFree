import Foundation

/// The 1€ filter (Casiez et al., 2012): low jitter at rest, low lag when moving fast.
/// Well suited to hand-tracking signals.
public struct OneEuroFilter: Sendable {
    public var minCutoff: Double
    public var beta: Double
    public var derivativeCutoff: Double

    private var previous: Double?
    private var previousDerivative = 0.0
    private var previousTime: Double?

    public init(minCutoff: Double = 1.0, beta: Double = 0.3, derivativeCutoff: Double = 1.0) {
        self.minCutoff = minCutoff
        self.beta = beta
        self.derivativeCutoff = derivativeCutoff
    }

    public mutating func filter(_ x: Double, at time: Double) -> Double {
        guard let prev = previous, let prevTime = previousTime else {
            previous = x
            previousTime = time
            return x
        }
        let dt = time - prevTime
        guard dt > 0 else { return prev }

        let dx = (x - prev) / dt
        let aD = Self.alpha(cutoff: derivativeCutoff, dt: dt)
        let dxHat = aD * dx + (1 - aD) * previousDerivative

        let cutoff = minCutoff + beta * abs(dxHat)
        let a = Self.alpha(cutoff: cutoff, dt: dt)
        let y = a * x + (1 - a) * prev

        previous = y
        previousDerivative = dxHat
        previousTime = time
        return y
    }

    public mutating func reset() {
        previous = nil
        previousTime = nil
        previousDerivative = 0
    }

    private static func alpha(cutoff: Double, dt: Double) -> Double {
        let tau = 1 / (2 * Double.pi * cutoff)
        return 1 / (1 + tau / dt)
    }
}
