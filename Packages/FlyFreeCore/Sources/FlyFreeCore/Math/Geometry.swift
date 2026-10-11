import simd

public typealias Vec3 = SIMD3<Double>
public typealias Quat = simd_quatd

/// Body-frame axes convention used throughout FlyFree (matches RealityKit):
/// +X = right, +Y = up, -Z = forward (nose). Rotations follow the right-hand rule,
/// so +X rotation = pitch up, +Y rotation = yaw left, +Z rotation = roll left.
public enum BodyAxis {
    public static let right = Vec3(1, 0, 0)
    public static let up = Vec3(0, 1, 0)
    public static let forward = Vec3(0, 0, -1)
}

/// A combined force + torque, usually expressed in the craft body frame.
public struct Wrench: Sendable, Equatable {
    public var force: Vec3
    public var torque: Vec3

    public init(force: Vec3 = .zero, torque: Vec3 = .zero) {
        self.force = force
        self.torque = torque
    }

    public static let zero = Wrench()

    public static func + (a: Wrench, b: Wrench) -> Wrench {
        Wrench(force: a.force + b.force, torque: a.torque + b.torque)
    }

    public static func - (a: Wrench, b: Wrench) -> Wrench {
        Wrench(force: a.force - b.force, torque: a.torque - b.torque)
    }

    public static func * (a: Wrench, s: Double) -> Wrench {
        Wrench(force: a.force * s, torque: a.torque * s)
    }

    /// Flattened as [Fx, Fy, Fz, Tx, Ty, Tz].
    var components: [Double] {
        [force.x, force.y, force.z, torque.x, torque.y, torque.z]
    }

    init(components c: [Double]) {
        self.init(force: Vec3(c[0], c[1], c[2]), torque: Vec3(c[3], c[4], c[5]))
    }
}

extension Comparable {
    func clamped(to range: ClosedRange<Self>) -> Self {
        min(max(self, range.lowerBound), range.upperBound)
    }
}

/// Unsigned angle between two vectors in radians; 0 if either is degenerate.
func angleBetween(_ a: Vec3, _ b: Vec3) -> Double {
    let la = simd_length(a), lb = simd_length(b)
    guard la > 1e-9, lb > 1e-9 else { return 0 }
    return acos((simd_dot(a, b) / (la * lb)).clamped(to: -1...1))
}

extension Quat {
    /// Builds a rotation from intrinsic yaw (Y), pitch (X), roll (Z), in radians.
    public static func fromYawPitchRoll(yaw: Double, pitch: Double, roll: Double) -> Quat {
        Quat(angle: yaw, axis: BodyAxis.up)
            * Quat(angle: pitch, axis: BodyAxis.right)
            * Quat(angle: roll, axis: Vec3(0, 0, 1))
    }
}
