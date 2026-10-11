import Testing
import Foundation
@testable import FlyFreeCore

@Suite struct SensitivityTests {
    let curve = AxisShaping.stick

    @Test func exponentialCurveIsFineNearCenterAndFullAtNinetyDegrees() {
        func response(degrees: Double) -> Double { curve.apply(degrees / 90) }
        #expect(response(degrees: 0) == 0)
        #expect(abs(response(degrees: 90) - 1) < 1e-12)
        #expect(response(degrees: 10) < 0.04)      // tiny motions → tiny commands
        #expect(response(degrees: 30) < 0.15)
        #expect(response(degrees: 60) < 0.45)
        #expect(response(degrees: -45) == -response(degrees: 45))
        var previous = 0.0
        for d in stride(from: 2.0, through: 90, by: 2) {
            let r = response(degrees: d)
            #expect(r > previous)
            previous = r
        }
    }
}

@Suite struct GaugeScaleTests {
    @Test func linearScaleTicksAndFractions() {
        let g = GaugeScale.jetAirspeed
        #expect(g.fraction(for: 175) == 0.5)
        #expect(g.fraction(for: 500) == 1)
        #expect(g.ticks.filter(\.major).compactMap(\.label) == ["0", "50", "100", "150", "200", "250", "300", "350"])
        #expect(g.ticks.count == 36)
    }

    @Test func logarithmicScaleSpansMetresToHundredsOfKilometresPerSecond() {
        let g = GaugeScale.spacecraft
        #expect(g.maxValue == 100_000)
        #expect(g.fraction(for: 0) == 0)
        #expect(abs(g.fraction(for: 10) - 0.12) < 1e-12)
        #expect(abs(g.fraction(for: 1000) - (0.12 + 0.88 / 2)) < 1e-12)
        #expect(g.fraction(for: 120_000) == 1)
        #expect(g.ticks.filter(\.major).compactMap(\.label) == ["0", "10", "100", "1k", "10k", "100k"])
        let fractions = g.ticks.map(\.fraction)
        #expect(fractions == fractions.sorted())
    }
}
