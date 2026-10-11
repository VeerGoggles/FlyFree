import Foundation
import simd

/// A target orientation for one hand's plane, in the pilot's frame (+X right, +Y up, −Z forward).
public struct HandPose: Sendable, Equatable {
    /// Direction the back of the hand faces.
    public var backOfHand: Vec3
    /// Direction the fingers point.
    public var fingers: Vec3
    /// Short instruction shown during calibration.
    public var instruction: String

    public init(backOfHand: Vec3, fingers: Vec3, instruction: String) {
        self.backOfHand = simd_normalize(backOfHand)
        self.fingers = simd_normalize(fingers)
        self.instruction = instruction
    }

    /// The hand-frame orientation for this pose, for a hand of the given chirality.
    /// (Hand frame: X lateral toward the user's right for a palm-down hand, Y back of hand, −Z fingers.)
    public func orientation(for chirality: Chirality) -> Quat {
        let n = backOfHand, f = fingers
        let l = simd_normalize(simd_cross(f, n))
        _ = chirality  // the plane fit already gives both hands the same frame for mirrored poses
        return Quat(simd_double3x3(columns: (l, n, -f))).normalized
    }

    /// This pose mirrored left ↔ right (for left-handed pilots, and for the other hand).
    public var mirrored: HandPose {
        HandPose(backOfHand: Vec3(-backOfHand.x, backOfHand.y, backOfHand.z),
                 fingers: Vec3(-fingers.x, fingers.y, fingers.z), instruction: instruction)
    }

    /// Flat, palm down, fingers forward.
    public static let flat = HandPose(backOfHand: Vec3(0, 1, 0), fingers: Vec3(0, 0, -1),
                                      instruction: "Flat, palm down, fingers forward")

    /// Flat, palm down, bent at the wrist so the fingertips point `angle` (rad) above forward —
    /// like pushing on something. Negative bends the fingertips down.
    public static func tipped(_ angle: Double, instruction: String) -> HandPose {
        HandPose(backOfHand: Vec3(0, cos(angle), sin(angle)), fingers: Vec3(0, sin(angle), -cos(angle)),
                 instruction: instruction)
    }
}

/// The hand poses a simulation starts from, by role. Written for a right-handed pilot (stick in
/// the right hand); `poses(dominant:)` mirrors them for a left-handed pilot.
public struct StartingPose: Sendable, Equatable {
    public var stick: HandPose
    public var throttle: HandPose
    /// Angle (rad) within which a hand counts as matching its pose.
    public var tolerance: Double

    public init(stick: HandPose, throttle: HandPose, tolerance: Double = 0.6) {
        self.stick = stick
        self.throttle = throttle
        self.tolerance = tolerance
    }

    /// Target pose for each physical hand.
    public func poses(dominant: Chirality) -> [Chirality: HandPose] {
        dominant == .right
            ? [.right: stick, .left: throttle]
            : [.left: stick.mirrored, .right: throttle.mirrored]
    }

    /// Both hands flat, palms down, fingers forward (the spacecraft and the drone). FlyFree's
    /// hands are always flat — never turned sideways.
    public static let bothFlat = StartingPose(stick: .flat, throttle: .flat)

    /// The jet: both hands flat and palms down, the stick hand bent up at the wrist like a push,
    /// the throttle hand bent down.
    public static let stickUpThrottleDown = StartingPose(
        stick: .tipped(.pi / 4, instruction: "Palm down, fingertips bent up, like a push"),
        throttle: .tipped(-.pi / 4, instruction: "Palm down, fingertips bent down"))
}
