import ARKit
import FlyFreeCore
import Observation
import simd

/// Owns the ARKit session and converts `HandAnchor`s into platform-independent `HandFrame`s.
@MainActor
@Observable
final class HandTrackingService {
    enum Status: Equatable {
        case idle, running, unsupported, denied, failed(String)
    }

    private(set) var status = Status.idle
    private let session = ARKitSession()
    private let provider = HandTrackingProvider()

    func start() async {
        guard status != .running else { return }
        guard HandTrackingProvider.isSupported else {
            status = .unsupported
            return
        }
        let authorization = await session.requestAuthorization(for: [.handTracking])
        guard authorization[.handTracking] == .allowed else {
            status = .denied
            return
        }
        do {
            try await session.run([provider])
            status = .running
        } catch {
            status = .failed(error.localizedDescription)
        }
    }

    func stop() {
        session.stop()
        status = .idle
    }

    func latestHands() -> (left: HandFrame?, right: HandFrame?) {
        guard status == .running else { return (nil, nil) }
        let anchors = provider.latestAnchors
        return (anchors.leftHand.flatMap(Self.frame), anchors.rightHand.flatMap(Self.frame))
    }

    private static let chains: [(Finger, [HandSkeleton.JointName])] = [
        (.thumb, [.thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip]),
        (.index, [.indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip]),
        (.middle, [.middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip]),
        (.ring, [.ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip]),
        (.little, [.littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip]),
    ]

    private static func frame(_ anchor: HandAnchor) -> HandFrame? {
        guard anchor.isTracked, let skeleton = anchor.handSkeleton else { return nil }
        let origin = anchor.originFromAnchorTransform
        func position(_ name: HandSkeleton.JointName) -> Vec3 {
            let p = (origin * skeleton.joint(name).anchorFromJointTransform).columns.3
            return Vec3(Double(p.x), Double(p.y), Double(p.z))
        }
        var fingers: [Finger: [Vec3]] = [:]
        for (finger, joints) in chains {
            fingers[finger] = joints.map(position)
        }
        return HandFrame(
            chirality: anchor.chirality == .left ? .left : .right,
            wrist: position(.wrist),
            fingers: fingers
        )
    }
}
