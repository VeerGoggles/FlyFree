/// How throttle is interpreted.
public enum FlightAssistMode: String, Sendable, CaseIterable, Identifiable {
    /// Throttle = fraction of main-engine thrust (negative = retro). Pure Newtonian.
    case off
    /// Throttle = desired forward speed (negative = reverse). The controller uses engines and
    /// RCS to reach it and to null sideways drift. Easiest to fly.
    case velocityHold
    public var id: String { rawValue }
}

/// What the pilot wants the craft to do, independent of how the craft achieves it.
public struct DesiredCraftState: Sendable, Equatable {
    /// Commanded body angular rate (rad/s).
    public var angularRate: Vec3
    /// World attitude to hold, when the stick is centered or released. `nil` = pure rate command.
    public var attitude: Quat?
    public var assist: FlightAssistMode
    /// `.off`: thrust fraction -1...1 along body forward.
    public var thrustFraction: Double
    /// `.velocityHold`: desired body-frame velocity (m/s).
    public var bodyVelocity: Vec3
    /// Feed-forward: how fast the rate reference is changing (rad/s², body).
    public var angularAcceleration: Vec3 = .zero
    /// Feed-forward: how fast the velocity reference is changing (m/s², body).
    public var linearAcceleration: Vec3 = .zero
    /// Aircraft: wheel brakes and speed brake.
    public var brakes = false
    /// Aircraft: only the thrust axis is velocity-controlled (lift and weight do the rest).
    public var thrustAxisOnly = false

    public init(
        angularRate: Vec3 = .zero,
        attitude: Quat? = nil,
        assist: FlightAssistMode = .velocityHold,
        thrustFraction: Double = 0,
        bodyVelocity: Vec3 = .zero
    ) {
        self.angularRate = angularRate
        self.attitude = attitude
        self.assist = assist
        self.thrustFraction = thrustFraction
        self.bodyVelocity = bodyVelocity
    }
}
