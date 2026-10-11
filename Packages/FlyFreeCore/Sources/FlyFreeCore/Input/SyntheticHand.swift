import simd

/// Generates plausible `HandFrame`s from a few parameters. Used by unit tests and by the
/// app's "Simulated Hands" input source (the visionOS simulator has no hand tracking).
public enum SyntheticHand {
    /// - Parameters:
    ///   - orientation: hand orientation (identity = palm down, fingers pointing -Z).
    ///   - position: wrist position in world space.
    ///   - curl: per-finger curl, 0 (straight) ... 1 (fist). Missing fingers default to 0.
    public static func make(
        chirality: Chirality,
        orientation: Quat = Quat(ix: 0, iy: 0, iz: 0, r: 1),
        position: Vec3 = .zero,
        curl: [Finger: Double] = [:]
    ) -> HandFrame {
        // Right hand, palm down: thumb side is -X. Mirror for the left hand.
        let side: Double = chirality == .right ? 1 : -1
        let lateral: [Finger: Double] = [.index: -0.025, .middle: 0, .ring: 0.02, .little: 0.038]
        let segments = [0.04, 0.025, 0.02]
        let maxBendPerJoint = 1.4

        func world(_ local: Vec3) -> Vec3 { position + orientation.act(local) }

        var fingers: [Finger: [Vec3]] = [:]
        for finger in [Finger.index, .middle, .ring, .little] {
            let x = lateral[finger]! * side
            let metacarpal = Vec3(x, 0, -0.03)
            let knuckle = Vec3(x, 0, -0.09)
            var points = [metacarpal, knuckle]
            let bend = (curl[finger] ?? 0).clamped(to: 0...1) * maxBendPerJoint
            var angle = 0.0
            var p = knuckle
            for length in segments {
                angle += bend
                p += Vec3(0, -sin(angle), -cos(angle)) * length
                points.append(p)
            }
            fingers[finger] = points.map(world)
        }

        // Thumb: a short chain off the index side, curling across the palm.
        let thumbBend = (curl[.thumb] ?? 0).clamped(to: 0...1) * 0.9
        var thumb = [Vec3(-0.035 * side, -0.01, -0.03)]
        var angle = 0.0
        for length in [0.035, 0.03, 0.025] {
            angle += thumbBend
            let dir = Vec3(-0.5 * side * cos(angle) + side * sin(angle) * 0.8, -sin(angle) * 0.4, -0.85 * cos(angle))
            thumb.append(thumb.last! + simd_normalize(dir) * length)
        }
        fingers[.thumb] = thumb.map(world)

        return HandFrame(chirality: chirality, wrist: position, fingers: fingers)
    }
}
