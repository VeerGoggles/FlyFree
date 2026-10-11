import simd

/// Geometric measurements derived from a `HandFrame`.
public enum HandMetrics {
    /// Total flexion along a joint chain in radians (sum of bend angles at each interior joint).
    /// A straight finger is ~0; a full fist is roughly 4–4.5 rad for the long fingers.
    public static func flexion(_ chain: [Vec3]) -> Double {
        guard chain.count >= 3 else { return 0 }
        var total = 0.0
        for i in 1..<(chain.count - 1) {
            total += angleBetween(chain[i] - chain[i - 1], chain[i + 1] - chain[i])
        }
        return total
    }

    public static func flexion(_ hand: HandFrame, _ finger: Finger) -> Double {
        flexion(hand.fingers[finger] ?? [])
    }

    /// Orientation of the hand as a rotation from body axes to world.
    ///
    /// Mapped so that a hand held palm-down with fingers pointing away from the user has
    /// lateral → +X (user's right), back-of-hand → +Y, fingers → -Z. Left and right hands
    /// produce the same frame for mirrored poses, so no extra sign flips are needed later.
    ///
    /// - Parameter pointer: when true, "forward" follows the extended index finger
    ///   instead of the wrist → middle-knuckle line.
    public static func orientation(of hand: HandFrame, pointer: Bool = false) -> Quat? {
        guard let indexK = hand.knuckle(.index),
              let middleK = hand.knuckle(.middle),
              let littleK = hand.knuckle(.little) else { return nil }

        var forward = middleK - hand.wrist
        if pointer, let tip = hand.tip(.index) { forward = tip - indexK }
        // Index → little points toward the user's right for a palm-down right hand.
        var lateral = littleK - indexK
        if hand.chirality == .left { lateral = -lateral }

        guard simd_length(forward) > 1e-6 else { return nil }
        let f = simd_normalize(forward)
        let lOrtho = lateral - simd_dot(lateral, f) * f
        guard simd_length(lOrtho) > 1e-6 else { return nil }
        let l = simd_normalize(lOrtho)
        let u = simd_cross(l, f)
        return Quat(simd_double3x3(columns: (l, u, -f))).normalized
    }

    /// Pitch / yaw / roll (radians, right-hand rule about body X / Y / Z) of `current`
    /// relative to `neutral`.
    public static func relativeAngles(current: Quat, neutral: Quat) -> Vec3 {
        let rel = (neutral.inverse * current).normalized
        let fwd = rel.act(BodyAxis.forward)
        let lat = rel.act(BodyAxis.right)
        let pitch = atan2(fwd.y, (fwd.x * fwd.x + fwd.z * fwd.z).squareRoot())
        let yaw = atan2(-fwd.x, -fwd.z)
        // Roll: express the lateral axis in the yaw/pitch-removed frame.
        let unYawPitch = (Quat(angle: yaw, axis: BodyAxis.up) * Quat(angle: pitch, axis: BodyAxis.right)).inverse
        let latLocal = unYawPitch.act(lat)
        let roll = atan2(latLocal.y, latLocal.x)
        return Vec3(pitch, yaw, roll)
    }
}
