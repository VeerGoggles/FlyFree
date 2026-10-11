/// Normalized, craft-agnostic pilot command — the output of hand interpretation.
public struct PilotInput: Sendable, Equatable {
    /// Rotation command per body axis (pitch about X, yaw about Y, roll about Z), each in -1...1.
    public var rotation: Vec3
    /// Throttle pedal in -1...1: positive = accelerate (fingertips tipped down), negative = brake/reverse.
    public var throttle: Double
    /// Translation command per body axis in -1...1 (reserved for RCS strafing; zero for now).
    public var translation: Vec3
    /// True while the dominant hand is held open/flat and calibrated (dead-man switch).
    public var engaged: Bool
    /// True while the stick hand is closed: return the craft to its reset attitude.
    public var stickClosed: Bool
    /// True while the throttle hand is closed: brake to a full stop.
    public var throttleClosed: Bool
    public var dominantHandTracked: Bool
    public var throttleHandTracked: Bool

    public init(
        rotation: Vec3 = .zero,
        throttle: Double = 0,
        translation: Vec3 = .zero,
        engaged: Bool = false,
        stickClosed: Bool = false,
        throttleClosed: Bool = false,
        dominantHandTracked: Bool = false,
        throttleHandTracked: Bool = false
    ) {
        self.rotation = rotation
        self.throttle = throttle
        self.translation = translation
        self.engaged = engaged
        self.stickClosed = stickClosed
        self.throttleClosed = throttleClosed
        self.dominantHandTracked = dominantHandTracked
        self.throttleHandTracked = throttleHandTracked
    }

    public static let neutral = PilotInput()
}
