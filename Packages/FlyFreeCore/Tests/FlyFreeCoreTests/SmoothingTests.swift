import Testing
@testable import FlyFreeCore

@Suite struct SmoothingTests {
    /// Time (at 240 Hz) to cover 90% of a small step — the fine-control regime where the
    /// filter's resting time constant dominates.
    private func settleTime(_ filter: OneEuroFilter) -> Double {
        var f = filter
        let dt = 1.0 / 240, step = 0.02
        _ = f.filter(0, at: 0)
        var t = dt
        while f.filter(step, at: t) < 0.9 * step { t += dt }
        return t
    }

    @Test func stickSmoothingIsAboutFourTimesFasterThanBefore() {
        let before = settleTime(OneEuroFilter(minCutoff: 1.2, beta: 0.4, derivativeCutoff: 1))
        let now = settleTime(InterpreterConfiguration().rotationSmoothing)
        #expect(abs(before / now - 4) < 0.6)
    }
}
