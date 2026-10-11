/// Which procedural cockpit the app builds around the pilot.
public enum CockpitStyle: String, Sendable {
    case spacecraftCapsule
    case fighterCanopy
}

/// Everything needed to fly a craft: physical properties, actuator layout, handling, visuals.
/// Adding a new craft should be (mostly) data, not code.
public struct CraftDefinition: Sendable, Identifiable {
    public var id: String
    public var name: String
    public var summary: String
    public var mass: MassProperties
    public var actuators: [Actuator]
    public var profile: ControlProfile
    public var gains: ControllerGains
    public var allocationWeights: ControlAllocator.Weights
    public var cockpit: CockpitStyle
    /// Rough hull size (width, height, length) in meters, for visuals.
    public var hullSize: Vec3

    public init(
        id: String, name: String, summary: String,
        mass: MassProperties, actuators: [Actuator],
        profile: ControlProfile, gains: ControllerGains,
        allocationWeights: ControlAllocator.Weights,
        cockpit: CockpitStyle, hullSize: Vec3
    ) {
        self.id = id
        self.name = name
        self.summary = summary
        self.mass = mass
        self.actuators = actuators
        self.profile = profile
        self.gains = gains
        self.allocationWeights = allocationWeights
        self.cockpit = cockpit
        self.hullSize = hullSize
    }

    /// Total forward thrust from main engines (N).
    public var maxForwardThrust: Double {
        actuators.filter { $0.kind == .mainEngine }
            .reduce(0) { $0 + max(0, -$1.direction.z) * $1.maxForce }
    }
}

public enum CraftCatalog {
    public static let all: [CraftDefinition] = [genericSpacecraft]
}
