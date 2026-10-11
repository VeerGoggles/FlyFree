import Foundation
import simd

/// Air-relative flight condition, computed every physics step.
public struct AirData: Sendable, Equatable {
    /// True airspeed (m/s).
    public var airspeed: Double
    /// Angle of attack (rad): positive when the air meets the wing from below.
    public var alpha: Double
    /// Sideslip (rad): positive when the craft slides toward its right.
    public var beta: Double
    /// Dynamic pressure ½ρV² (Pa).
    public var dynamicPressure: Double
    /// Air-relative velocity in body axes (m/s).
    public var bodyAirVelocity: Vec3
    public var altitude: Double

    public static func compute(state: RigidBodyState, air: AirState) -> AirData {
        let va = state.orientation.inverse.act(state.velocity - air.wind)
        let v = simd_length(va)
        let alpha = v > 0.5 ? atan2(-va.y, -va.z) : 0
        let beta = v > 0.5 ? asin((va.x / v).clamped(to: -1...1)) : 0
        return AirData(airspeed: v, alpha: alpha, beta: beta, dynamicPressure: 0.5 * air.density * v * v,
                       bodyAirVelocity: va, altitude: state.position.y)
    }
}

/// Coefficient-based aerodynamic model of a conventional aircraft (body axes: +X right,
/// +Y up, −Z forward). Control-surface moments are *not* here — those are actuators, so the
/// allocator can use them; this is the bare airframe: lift, drag, side force and the stability
/// and damping moments that make an aircraft weathervane, pitch-stable and self-damping.
public struct AerodynamicModel: Sendable, Equatable {
    public var wingArea: Double      // S (m²)
    public var span: Double          // b (m)
    public var chord: Double         // c̄ (m)

    public var cl0: Double
    public var clAlpha: Double       // per rad
    public var stallAlpha: Double    // rad
    public var cd0: Double
    public var inducedDragFactor: Double
    public var speedBrakeDrag: Double
    public var sideForceBeta: Double // magnitude, per rad

    public var cm0: Double
    public var cmAlpha: Double       // negative = pitch-stable
    public var cmPitchDamping: Double
    public var rollDihedral: Double  // magnitude: slip rolls the leading wing up
    public var rollDamping: Double
    public var yawStability: Double  // magnitude: weathervane into the slip
    public var yawDamping: Double

    public init(wingArea: Double, span: Double, chord: Double, cl0: Double, clAlpha: Double, stallAlpha: Double,
                cd0: Double, inducedDragFactor: Double, speedBrakeDrag: Double, sideForceBeta: Double,
                cm0: Double, cmAlpha: Double, cmPitchDamping: Double, rollDihedral: Double, rollDamping: Double,
                yawStability: Double, yawDamping: Double) {
        self.wingArea = wingArea; self.span = span; self.chord = chord
        self.cl0 = cl0; self.clAlpha = clAlpha; self.stallAlpha = stallAlpha
        self.cd0 = cd0; self.inducedDragFactor = inducedDragFactor; self.speedBrakeDrag = speedBrakeDrag
        self.sideForceBeta = sideForceBeta
        self.cm0 = cm0; self.cmAlpha = cmAlpha; self.cmPitchDamping = cmPitchDamping
        self.rollDihedral = rollDihedral; self.rollDamping = rollDamping
        self.yawStability = yawStability; self.yawDamping = yawDamping
    }

    /// Lift coefficient with a linear range and a post-stall drop.
    public func liftCoefficient(_ alpha: Double) -> Double {
        let a = abs(alpha)
        guard a > stallAlpha else { return cl0 + clAlpha * alpha }
        let peak = alpha > 0 ? cl0 + clAlpha * stallAlpha : cl0 - clAlpha * stallAlpha
        return peak * max(0.45, 1 - 1.6 * (a - stallAlpha))
    }

    public var maxLiftCoefficient: Double { cl0 + clAlpha * stallAlpha }

    public struct Result: Sendable {
        public var bodyForce: Vec3
        public var bodyTorque: Vec3
        public var lift: Double
    }

    public func loads(_ air: AirData, angularVelocity w: Vec3, speedBrake: Bool) -> Result {
        let v = air.airspeed
        guard v > 1 else { return Result(bodyForce: .zero, bodyTorque: .zero, lift: 0) }
        let q = air.dynamicPressure
        let vHat = air.bodyAirVelocity / v

        let cl = liftCoefficient(air.alpha)
        let stallDrag = abs(air.alpha) > stallAlpha ? 1.2 * (abs(air.alpha) - stallAlpha) : 0
        let cd = cd0 + inducedDragFactor * cl * cl + stallDrag + (speedBrake ? speedBrakeDrag : 0)

        var liftDir = simd_cross(BodyAxis.right, vHat)
        liftDir = simd_length(liftDir) > 1e-6 ? simd_normalize(liftDir) : BodyAxis.up
        let lift = q * wingArea * cl
        let force = liftDir * lift - vHat * (q * wingArea * cd) + BodyAxis.right * (-q * wingArea * sideForceBeta * air.beta)

        let pitch = q * wingArea * chord * (cm0 + cmAlpha * air.alpha + cmPitchDamping * w.x * chord / (2 * v))
        let yaw = q * wingArea * span * (-yawStability * air.beta + yawDamping * w.y * span / (2 * v))
        let roll = q * wingArea * span * (rollDihedral * air.beta + rollDamping * w.z * span / (2 * v))
        return Result(bodyForce: force, bodyTorque: Vec3(pitch, yaw, roll), lift: lift)
    }
}
