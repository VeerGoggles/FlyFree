import Testing
import simd
@testable import FlyFreeCore

/// The spacecraft and drone start with both hands flat; the jet with both hands palm down, the
/// stick hand bent up and the throttle hand bent down. Hands are never turned sideways, and
/// calibration refuses a hand that isn't in its starting pose.
@Suite struct StartingPoseTests {
    private let identity = Quat(ix: 0, iy: 0, iz: 0, r: 1)
    private let sideways = Quat(angle: -.pi / 2, axis: Vec3(0, 0, 1))  // right hand turned thumb-up

    private func hand(_ c: Chirality, _ q: Quat) -> HandFrame {
        SyntheticHand.make(chirality: c, orientation: q, position: Vec3(c == .right ? 0.2 : -0.2, 1.0, -0.35))
    }

    private func calibrate(left: Quat, right: Quat, pose: StartingPose = .bothFlat) -> CalibrationProgress {
        var session = CalibrationSession(pose: pose)
        var progress = session.update(left: hand(.left, left), right: hand(.right, right), time: 0)
        var t = 0.0
        while progress.result == nil, t < 2.5 {
            t += 1.0 / 90
            progress = session.update(left: hand(.left, left), right: hand(.right, right), time: t)
        }
        return progress
    }

    @Test func spacecraftAndDroneStartWithBothHandsFlat() {
        #expect(CraftCatalog.genericSpacecraft.startingPose == .bothFlat)
        #expect(CraftCatalog.drone.startingPose == .bothFlat)
    }

    @Test func jetStartsWithStickBentUpAndThrottleBentDown() {
        let pose = CraftCatalog.jet.startingPose
        #expect(pose == .stickUpThrottleDown)
        #expect(pose.stick.fingers.y > 0.5)      // fingertips up, like a push
        #expect(pose.throttle.fingers.y < -0.5)  // fingertips down
        let down = Quat(angle: -.pi / 4, axis: BodyAxis.right), up = Quat(angle: .pi / 4, axis: BodyAxis.right)
        #expect(calibrate(left: down, right: up, pose: pose).result != nil)
        let flat = calibrate(left: identity, right: identity, pose: pose)
        #expect(flat.result == nil)
        #expect(flat.left == .wrongPose && flat.right == .wrongPose)
    }

    @Test(arguments: CraftCatalog.all.map(\.id))
    func noCraftTurnsAHandSideways(id: String) throws {
        let craft = try #require(CraftCatalog.all.first { $0.id == id })
        for chirality in Chirality.allCases {
            for pose in craft.startingPose.poses(dominant: chirality).values {
                #expect(pose.backOfHand.y > 0.7)   // palm down
                #expect(abs(pose.backOfHand.x) < 1e-9)
            }
        }
    }

    @Test func calibrationRefusesASidewaysHand() {
        let progress = calibrate(left: identity, right: sideways)
        #expect(progress.result == nil)
        #expect(progress.right == .wrongPose)
        #expect(calibrate(left: identity, right: identity).result != nil)
    }
}
