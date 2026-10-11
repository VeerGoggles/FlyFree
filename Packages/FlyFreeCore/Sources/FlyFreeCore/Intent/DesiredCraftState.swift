/// The frame the stick's rotation is expressed in.
public enum ControlFrame: String, Sendable, CaseIterable, Identifiable {
    /// The global (launch) frame. Tipping your fingertips up always rotates the ship about the
    /// same world axis — the one your hand rotated about — whatever the ship's current attitude.
    /// The command is re-projected into the ship's current frame with its quaternion every step,
    /// so the hand, the attitude hologram and the ship always agree.
    case global
    /// The ship's own body axes, like a conventional cockpit stick: "pitch up" is always nose-up
    /// relative to the pilot's seat.
    case body
    public var id: String { rawValue }
}

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
