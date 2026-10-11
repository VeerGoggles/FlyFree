import Testing
import simd
@testable import FlyFreeCore

/// Every craft starts with both hands flat — never turned sideways — and calibration refuses a
/// hand that isn't.
@Suite struct StartingPoseTests {
    private let identity = Quat(ix: 0, iy: 0, iz: 0, r: 1)
    private let sideways = Quat(angle: -.pi / 2, axis: Vec3(0, 0, 1))  // right hand turned thumb-up

    private func hand(_ c: Chirality, _ q: Quat) -> HandFrame {
        SyntheticHand.make(chirality: c, orientation: q, position: Vec3(c == .right ? 0.2 : -0.2, 1.0, -0.35))
    }

    private func calibrate(left: Quat, right: Quat) -> CalibrationProgress {
        var session = CalibrationSession(pose: .bothFlat)
        var progress = session.update(left: hand(.left, left), right: hand(.right, right), time: 0)
        var t = 0.0
        while progress.result == nil, t < 2.5 {
            t += 1.0 / 90
            progress = session.update(left: hand(.left, left), right: hand(.right, right), time: t)
        }
        return progress
    }

    @Test(arguments: CraftCatalog.all.map(\.id))
    func everyCraftStartsWithBothHandsFlat(id: String) throws {
        let craft = try #require(CraftCatalog.all.first { $0.id == id })
        #expect(craft.startingPose == .bothFlat)
        for chirality in Chirality.allCases {
            let up = craft.startingPose.poses(dominant: chirality).values.map(\.backOfHand)
            for normal in up { #expect(normal.y > 0.99) }
        }
    }

    @Test func calibrationRefusesASidewaysHand() {
        let progress = calibrate(left: identity, right: sideways)
        #expect(progress.result == nil)
        #expect(progress.right == .wrongPose)
        #expect(calibrate(left: identity, right: identity).result != nil)
    }
}
