import simd

/// A plane fitted through the finger joints of a (mostly) flat hand, with a full
/// orientation frame. This is the stick: fitting ~20 joints is far more robust to
/// tracking noise and occlusion than reading any single finger.
public struct HandPlane: Sendable, Equatable {
    /// Mean of the fitted joints (world, m).
    public var centroid: Vec3
    /// Hand frame → world. Matches the body convention for a palm-down hand:
    /// X = lateral (user's right), Y = back-of-hand normal, -Z = toward the fingertips.
    public var orientation: Quat
    /// RMS distance of the joints from the plane (m). ~2–5 mm flat, 15 mm+ curled.
    public var rmsResidual: Double

    public init(centroid: Vec3, orientation: Quat, rmsResidual: Double) {
        self.centroid = centroid
        self.orientation = orientation
        self.rmsResidual = rmsResidual
    }

    public var normal: Vec3 { orientation.act(BodyAxis.up) }
    public var forward: Vec3 { orientation.act(BodyAxis.forward) }
}

/// Geometric measurements derived from a `HandFrame`.
public enum HandMetrics {
    static let planeFingers: [Finger] = [.index, .middle, .ring, .little]

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

    /// Mean normalized curl (0 = flat, 1 = fist) of the four long fingers.
    public static func curl(_ hand: HandFrame, calibration: CurlCalibration = .init()) -> Double {
        let curls = planeFingers.map { calibration.normalize(flexion(hand, $0)) }
        return curls.reduce(0, +) / Double(curls.count)
    }

    /// Least-squares plane through every joint of the four long fingers, oriented as a hand frame.
    public static func plane(of hand: HandFrame) -> HandPlane? {
        let chains = planeFingers.compactMap { hand.fingers[$0] }
        guard chains.count == planeFingers.count,
              let indexK = hand.knuckle(.index), let littleK = hand.knuckle(.little) else { return nil }
        let points = chains.flatMap { $0 }
        guard points.count >= 8 else { return nil }

        let centroid = points.reduce(Vec3.zero, +) / Double(points.count)
        var covariance = simd_double3x3()
        for p in points {
            let d = p - centroid
            covariance += simd_double3x3(columns: (d * d.x, d * d.y, d * d.z))
        }
        var normal = smallestEigenvector(of: covariance)

        // Disambiguate directions with the coarse hand layout.
        let proximal = chains.map { $0.first! }.reduce(Vec3.zero, +) / Double(chains.count)
        let distal = chains.map { $0.last! }.reduce(Vec3.zero, +) / Double(chains.count)
        let forwardRaw = distal - proximal
        var lateralRaw = littleK - indexK
        if hand.chirality == .left { lateralRaw = -lateralRaw }
        if simd_dot(normal, simd_cross(lateralRaw, forwardRaw)) < 0 { normal = -normal }

        let forwardInPlane = forwardRaw - simd_dot(forwardRaw, normal) * normal
        guard simd_length(forwardInPlane) > 1e-6 else { return nil }
        let f = simd_normalize(forwardInPlane)
        let l = simd_cross(f, normal)
        let orientation = Quat(simd_double3x3(columns: (l, normal, -f))).normalized

        let residual = (points.reduce(0) { $0 + pow(simd_dot($1 - centroid, normal), 2) } / Double(points.count)).squareRoot()
        return HandPlane(centroid: centroid, orientation: orientation, rmsResidual: residual)
    }

    /// Rotation of `current` relative to `neutral`, as a rotation vector (axis × angle, radians)
    /// in the neutral hand frame: x = pitch (fingertips up/down), y = yaw (wrist turned
    /// left/right), z = roll (forearm rotation). Coupling-free and well defined in all directions.
    public static func rotationVector(current: Quat, neutral: Quat) -> Vec3 {
        var rel = (neutral.inverse * current).normalized
        if rel.real < 0 { rel = Quat(vector: -rel.vector) }
        let angle = rel.angle
        guard angle > 1e-9 else { return .zero }
        return rel.axis * angle
    }

    /// Rotation from `neutral` to `current` as a rotation vector in the **pilot's frame**
    /// (+X right, +Y up, −Z forward), whatever pose the hand started in. Tilting a vertical
    /// "stick" hand forward is a rotation about +X, exactly like tipping a flat hand.
    public static func pilotRotationVector(current: Quat, neutral: Quat) -> Vec3 {
        rotationVector(current: (current * neutral.inverse).normalized, neutral: Quat(ix: 0, iy: 0, iz: 0, r: 1))
    }

    /// Signed rotation of `current` relative to `neutral` about one axis of the neutral frame,
    /// via swing–twist decomposition of the relative quaternion (no Euler angles, no gimbal lock).
    public static func twistAngle(current: Quat, neutral: Quat, about axis: Vec3) -> Double {
        var rel = (neutral.inverse * current).normalized
        if rel.real < 0 { rel = Quat(vector: -rel.vector) }
        let a = simd_normalize(axis)
        return 2 * atan2(simd_dot(rel.imag, a), rel.real)
    }

    /// Eigenvector of the smallest eigenvalue of a symmetric 3×3 matrix (cyclic Jacobi).
    static func smallestEigenvector(of matrix: simd_double3x3) -> Vec3 {
        var a = [[Double]](repeating: [0, 0, 0], count: 3)
        for c in 0..<3 { for r in 0..<3 { a[r][c] = matrix[c][r] } }
        var v: [[Double]] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for _ in 0..<32 {
            let off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2]
            if off < 1e-30 { break }
            for (p, q) in [(0, 1), (0, 2), (1, 2)] where abs(a[p][q]) > 1e-300 {
                let theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
                let t = (theta >= 0 ? 1 : -1) / (abs(theta) + (theta * theta + 1).squareRoot())
                let c = 1 / (t * t + 1).squareRoot(), s = t * c
                for k in 0..<3 {
                    let akp = a[k][p], akq = a[k][q]
                    a[k][p] = c * akp - s * akq
                    a[k][q] = s * akp + c * akq
                }
                for k in 0..<3 {
                    let apk = a[p][k], aqk = a[q][k]
                    a[p][k] = c * apk - s * aqk
                    a[q][k] = s * apk + c * aqk
                }
                for k in 0..<3 {
                    let vkp = v[k][p], vkq = v[k][q]
                    v[k][p] = c * vkp - s * vkq
                    v[k][q] = s * vkp + c * vkq
                }
            }
        }
        let i = (0..<3).min { a[$0][$0] < a[$1][$1] }!
        return simd_normalize(Vec3(v[0][i], v[1][i], v[2][i]))
    }
}
