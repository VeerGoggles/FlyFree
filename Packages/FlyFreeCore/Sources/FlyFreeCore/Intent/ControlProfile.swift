/// Deadzone + expo shaping for one normalized axis.
public struct AxisShaping: Sendable, Equatable {
    public var deadzone: Double
    /// 0 = linear, 1 = fully cubic (fine control near center).
    public var expo: Double

    public init(deadzone: Double = 0.06, expo: Double = 0.35) {
        self.deadzone = deadzone
        self.expo = expo
    }

    public func apply(_ x: Double) -> Double {
        let magnitude = abs(x)
        guard magnitude > deadzone else { return 0 }
        let n = ((magnitude - deadzone) / (1 - deadzone)).clamped(to: 0...1)
        let shaped = (1 - expo) * n + expo * n * n * n
        return x < 0 ? -shaped : shaped
    }
}

/// How the craft's handling feels: how far and how fast a full deflection takes you.
public struct ControlProfile: Sendable, Equatable {
    /// Body angular rate (rad/s) commanded at full deflection, per axis (pitch, yaw, roll).
    public var maxAngularRate: Vec3
    public var rotationShaping: AxisShaping
    public var throttleShaping: AxisShaping
    /// Forward speed (m/s) commanded at full throttle in velocity-hold flight assist.
    public var maxForwardSpeed: Double
    /// Lateral speed (m/s) commanded at full translation input in velocity-hold flight assist.
    public var maxStrafeSpeed: Double
    public var invertPitch: Bool

    public init(
        maxAngularRate: Vec3,
        rotationShaping: AxisShaping = .init(),
        throttleShaping: AxisShaping = .init(deadzone: 0.04, expo: 0.2),
        maxForwardSpeed: Double,
        maxStrafeSpeed: Double,
        invertPitch: Bool = false
    ) {
        self.maxAngularRate = maxAngularRate
        self.rotationShaping = rotationShaping
        self.throttleShaping = throttleShaping
        self.maxForwardSpeed = maxForwardSpeed
        self.maxStrafeSpeed = maxStrafeSpeed
        self.invertPitch = invertPitch
    }
}
