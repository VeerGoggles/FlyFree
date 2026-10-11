import simd

/// The neutral ("stick centered") orientation — and position — of each hand's plane, captured
/// while the pilot holds the starting pose. All control is measured relative to it.
public struct HandCalibration: Sendable, Equatable {
    public var left: Quat
    public var right: Quat
    /// Neutral hand-plane centres (world, m), for controls driven by where a hand is.
    public var leftPosition: Vec3?
    public var rightPosition: Vec3?

    public init(left: Quat, right: Quat, leftPosition: Vec3? = nil, rightPosition: Vec3? = nil) {
        self.left = left
        self.right = right
        self.leftPosition = leftPosition
        self.rightPosition = rightPosition
    }

    public subscript(_ chirality: Chirality) -> Quat {
        chirality == .left ? left : right
    }

    public func position(_ chirality: Chirality) -> Vec3? {
        chirality == .left ? leftPosition : rightPosition
    }

    /// Both hands palm-down, fingers pointing -Z.
    public static let identity = HandCalibration(left: Quat(ix: 0, iy: 0, iz: 0, r: 1), right: Quat(ix: 0, iy: 0, iz: 0, r: 1))
}

/// Why a hand is or isn't ready to be captured as neutral.
public enum HandReadiness: String, Sendable {
    case notDetected
    case notFlat      // fingers curled or spread out of plane
    case wrongPose    // hand not in this simulation's starting pose
    case moving
    case ready
}

public struct CalibrationProgress: Sendable, Equatable {
    public var left: HandReadiness
    public var right: HandReadiness
    /// 0...1 progress of the steady hold.
    public var fraction: Double
    public var result: HandCalibration?

    public init(left: HandReadiness, right: HandReadiness, fraction: Double, result: HandCalibration? = nil) {
        self.left = left
        self.right = right
        self.fraction = fraction
        self.result = result
    }
}

/// "Hold out both hands, flat" — like the Vision Pro hand setup. Completes once both hands
/// are flat, palm-down and steady for `holdDuration`, averaging their orientation over the hold.
public struct CalibrationSession: Sendable {
    public struct Requirements: Sendable, Equatable {
        public var holdDuration = 1.5
        public var maxCurl = 0.3
        public var maxResidual = 0.012
        /// Max hand angular speed (rad/s) to count as steady.
        public var maxAngularSpeed = 0.5
        public init() {}
    }

    public var requirements: Requirements
    public var curlCalibration: CurlCalibration
    /// Target pose per hand; nil = flat, palm down.
    public var poses: [Chirality: HandPose]
    public var poseTolerance: Double

    private var holdStart: Double?
    private var sums: [Chirality: SIMD4<Double>] = [:]
    private var positionSums: [Chirality: Vec3] = [:]
    private var samples = 0
    private var previous: [Chirality: (Quat, Double)] = [:]
    private var result: HandCalibration?

    public init(requirements: Requirements = .init(), curlCalibration: CurlCalibration = .init(),
                pose: StartingPose = .bothFlat, dominant: Chirality = .right) {
        self.requirements = requirements
        self.curlCalibration = curlCalibration
        self.poses = pose.poses(dominant: dominant)
        self.poseTolerance = pose.tolerance
    }

    public mutating func reset() {
        holdStart = nil
        sums = [:]
        positionSums = [:]
        samples = 0
        previous = [:]
        result = nil
    }

    public mutating func update(left: HandFrame?, right: HandFrame?, time: Double) -> CalibrationProgress {
        if let result {
            return CalibrationProgress(left: .ready, right: .ready, fraction: 1, result: result)
        }
        let leftPlane = left.flatMap(HandMetrics.plane)
        let rightPlane = right.flatMap(HandMetrics.plane)
        let l = readiness(.left, left, leftPlane, time)
        let r = readiness(.right, right, rightPlane, time)

        guard l == .ready, r == .ready, let leftPlane, let rightPlane else {
            holdStart = nil
            sums = [:]
        positionSums = [:]
        samples = 0
            return CalibrationProgress(left: l, right: r, fraction: 0)
        }

        let start = holdStart ?? time
        holdStart = start
        accumulate(.left, leftPlane.orientation)
        accumulate(.right, rightPlane.orientation)
        positionSums[.left, default: .zero] += leftPlane.centroid
        positionSums[.right, default: .zero] += rightPlane.centroid
        samples += 1

        let fraction = ((time - start) / requirements.holdDuration).clamped(to: 0...1)
        if fraction >= 1, let ls = sums[.left], let rs = sums[.right] {
            let n = Double(max(samples, 1))
            result = HandCalibration(left: Quat(vector: simd_normalize(ls)), right: Quat(vector: simd_normalize(rs)),
                                     leftPosition: positionSums[.left].map { $0 / n },
                                     rightPosition: positionSums[.right].map { $0 / n })
        }
        return CalibrationProgress(left: l, right: r, fraction: fraction, result: result)
    }

    private mutating func readiness(_ chirality: Chirality, _ hand: HandFrame?, _ plane: HandPlane?, _ time: Double) -> HandReadiness {
        guard let hand, let plane else {
            previous[chirality] = nil
            return .notDetected
        }
        defer { previous[chirality] = (plane.orientation, time) }
        if HandMetrics.curl(hand, calibration: curlCalibration) > requirements.maxCurl
            || plane.rmsResidual > requirements.maxResidual {
            return .notFlat
        }
        let target = (poses[chirality] ?? .flat).orientation(for: chirality)
        if simd_length(HandMetrics.rotationVector(current: plane.orientation, neutral: target)) > poseTolerance {
            return .wrongPose
        }
        if let (q, t) = previous[chirality], time > t {
            let speed = simd_length(HandMetrics.rotationVector(current: plane.orientation, neutral: q)) / (time - t)
            if speed > requirements.maxAngularSpeed { return .moving }
        }
        return .ready
    }

    /// Quaternion averaging by sign-aligned summation (accurate for small spreads).
    private mutating func accumulate(_ chirality: Chirality, _ q: Quat) {
        var v = q.vector
        if let sum = sums[chirality], simd_dot(sum, v) < 0 { v = -v }
        sums[chirality, default: .zero] += v
    }
}
