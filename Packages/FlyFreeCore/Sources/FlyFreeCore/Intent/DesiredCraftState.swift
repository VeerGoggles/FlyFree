/// How throttle is interpreted.
public enum FlightAssistMode: String, Sendable, CaseIterable, Identifiable {
    /// Throttle = fraction of main-engine thrust. Pure Newtonian; you must flip to brake.
    case off
    /// Throttle = desired forward speed. The controller uses engines and RCS to reach it
    /// and to null sideways drift. Easiest to fly.
    case velocityHold
    public var id: String { rawValue }
}

/// What the pilot wants the craft to do, independent of how the craft achieves it.
public struct DesiredCraftState: Sendable, Equatable {
    /// Body angular rate (rad/s). Zero = hold current attitude.
    public var angularRate: Vec3
    public var assist: FlightAssistMode
    /// `.off`: thrust fraction 0...1 along body forward.
    public var thrustFraction: Double
    /// `.velocityHold`: desired body-frame velocity (m/s).
    public var bodyVelocity: Vec3

    public init(
        angularRate: Vec3 = .zero,
        assist: FlightAssistMode = .velocityHold,
        thrustFraction: Double = 0,
        bodyVelocity: Vec3 = .zero
    ) {
        self.angularRate = angularRate
        self.assist = assist
        self.thrustFraction = thrustFraction
        self.bodyVelocity = bodyVelocity
    }
}
