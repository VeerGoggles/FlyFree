import Foundation

/// Deadzone + response curve for one normalized axis.
public struct AxisShaping: Sendable, Equatable {
    public var deadzone: Double
    /// Cubic blend: 0 = linear, 1 = fully cubic. Ignored when `exponential` > 0.
    public var expo: Double
    /// Exponential curve steepness k: response = (e^(k·n) − 1) / (e^k − 1). Very fine control
    /// near center, reaching full value only at full deflection. 0 = use the cubic blend.
    public var exponential: Double

    public init(deadzone: Double = 0.06, expo: Double = 0.35, exponential: Double = 0) {
        self.deadzone = deadzone
        self.expo = expo
        self.exponential = exponential
    }

    /// Stick default: exponential, full value only at ±90° of hand rotation.
    public static let stick = AxisShaping(deadzone: 0.02, expo: 0, exponential: 2.5)

    public func apply(_ x: Double) -> Double {
        let magnitude = abs(x)
        guard magnitude > deadzone else { return 0 }
        let n = ((magnitude - deadzone) / (1 - deadzone)).clamped(to: 0...1)
        let shaped = exponential > 0
            ? (exp(exponential * n) - 1) / (exp(exponential) - 1)
            : (1 - expo) * n + expo * n * n * n
        return x < 0 ? -shaped : shaped
    }
}

/// How the craft's handling feels. Rotation and translation have separate, independent
/// acceleration limits, and braking is deliberately much stronger than speeding up, so the
/// craft eases into motion but settles quickly (low "momentum").
public struct ControlProfile: Sendable, Equatable {
    /// Body angular rate (rad/s) commanded at full deflection, per axis (pitch, yaw, roll).
    public var maxAngularRate: Vec3
    /// How quickly commanded rotation builds up (rad/s², per axis).
    public var angularAcceleration: Vec3
    /// How quickly rotation is taken out when the stick returns toward center (rad/s², per axis).
    public var angularDeceleration: Vec3
    /// Forward acceleration (m/s²) at full throttle.
    public var linearAcceleration: Double
    /// Braking (m/s²): full reverse throttle, closed throttle hand, and nulling drift.
    public var linearDeceleration: Double
    /// Closed stick hand: attitude error (rad) → return rate (rad/s) toward the reset attitude.
    public var levelingGain: Double
    public var rotationShaping: AxisShaping
    public var throttleShaping: AxisShaping
    /// Top forward speed (m/s) in velocity-hold flight assist.
    public var maxForwardSpeed: Double
    /// Lateral speed (m/s) commanded at full translation input in velocity-hold flight assist.
    public var maxStrafeSpeed: Double
    public var invertPitch: Bool

    public init(
        maxAngularRate: Vec3,
        angularAcceleration: Vec3,
        angularDeceleration: Vec3,
        linearAcceleration: Double,
        linearDeceleration: Double,
        levelingGain: Double = 1.0,
        rotationShaping: AxisShaping = .stick,
        throttleShaping: AxisShaping = .init(deadzone: 0.06, expo: 0.2),
        maxForwardSpeed: Double,
        maxStrafeSpeed: Double,
        invertPitch: Bool = false
    ) {
        self.maxAngularRate = maxAngularRate
        self.angularAcceleration = angularAcceleration
        self.angularDeceleration = angularDeceleration
        self.linearAcceleration = linearAcceleration
        self.linearDeceleration = linearDeceleration
        self.levelingGain = levelingGain
        self.rotationShaping = rotationShaping
        self.throttleShaping = throttleShaping
        self.maxForwardSpeed = maxForwardSpeed
        self.maxStrafeSpeed = maxStrafeSpeed
        self.invertPitch = invertPitch
    }
}
