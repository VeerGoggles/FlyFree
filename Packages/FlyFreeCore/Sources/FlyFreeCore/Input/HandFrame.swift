import simd

public enum Chirality: String, Sendable, Codable, CaseIterable, Identifiable {
    case left, right
    public var id: String { rawValue }
    public var opposite: Chirality { self == .left ? .right : .left }
}

public enum Finger: Int, Sendable, CaseIterable {
    case thumb, index, middle, ring, little
}

/// A platform-independent snapshot of one tracked hand, in world space (meters).
///
/// The visionOS app fills this from ARKit's `HandAnchor`; tests and the simulator
/// fill it from `SyntheticHand`. Keeping the core free of ARKit makes the whole
/// control pipeline testable on macOS.
public struct HandFrame: Sendable, Equatable {
    public var chirality: Chirality
    public var wrist: Vec3
    /// Joint positions per finger, ordered proximal → distal.
    /// Fingers: metacarpal, knuckle, intermediateBase, intermediateTip, tip.
    /// Thumb: knuckle, intermediateBase, intermediateTip, tip.
    public var fingers: [Finger: [Vec3]]

    public init(chirality: Chirality, wrist: Vec3, fingers: [Finger: [Vec3]]) {
        self.chirality = chirality
        self.wrist = wrist
        self.fingers = fingers
    }

    /// The knuckle (MCP joint) of a finger, if present.
    public func knuckle(_ finger: Finger) -> Vec3? {
        guard let chain = fingers[finger] else { return nil }
        return finger == .thumb ? chain.first : (chain.count > 1 ? chain[1] : nil)
    }

    public func tip(_ finger: Finger) -> Vec3? { fingers[finger]?.last }
}
