/// Which world the craft flies in (physics environment and scenery).
public enum WorldKind: String, Sendable {
    case deepSpace
    case earth
    /// Earth physics with a city to fly through.
    case city
}

/// Flight computer for multirotors: the stick flies velocity relative to the craft's heading
/// (tilt to move, twist to yaw), the pedal flies climb rate, and letting go hovers in place.
public struct MultirotorControl: Sendable, Equatable {
    public var maxHorizontalSpeed: Double
    public var maxClimbRate: Double
    public var maxDescentRate: Double
    public var maxYawRate: Double
    /// Maximum tilt from level (rad); also caps horizontal acceleration at g·tan(tilt).
    public var maxTilt: Double
    /// Velocity loop gain (1/s) and altitude-hold gain (1/s).
    public var velocityGain = 1.6
    public var altitudeGain = 1.0

    public init(maxHorizontalSpeed: Double, maxClimbRate: Double, maxDescentRate: Double,
                maxYawRate: Double, maxTilt: Double) {
        self.maxHorizontalSpeed = maxHorizontalSpeed
        self.maxClimbRate = maxClimbRate
        self.maxDescentRate = maxDescentRate
        self.maxYawRate = maxYawRate
        self.maxTilt = maxTilt
    }
}

/// Bluff-body air drag and rotational damping (for craft without a lifting-wing model).
public struct BodyDrag: Sendable, Equatable {
    /// Drag coefficient × reference area (m²).
    public var dragArea: Double
    /// Rotational damping (N·m per rad/s), per body axis.
    public var angularDamping: Vec3

    public init(dragArea: Double, angularDamping: Vec3) {
        self.dragArea = dragArea
        self.angularDamping = angularDamping
    }
}

/// Flight-computer settings for aircraft (fly-by-wire on top of the rate controller).
public struct AircraftControl: Sendable, Equatable {
    /// Angle of attack the protection holds the wing below (rad).
    public var alphaLimit: Double
    /// Pitch-down rate (rad/s) per radian beyond the limit.
    public var alphaProtectionGain = 3.0
    /// Seconds of pitch rate added to α when checking the limit.
    public var alphaLookahead = 0.35
    /// Airspeed above which turns are auto-coordinated (m/s).
    public var coordinationSpeed = 45.0
    /// Autothrottle airspeed range (m/s).
    public var maxAirspeed: Double
    /// How far the autothrottle target may lead the actual airspeed (m/s).
    public var speedLead = 15.0

    public init(alphaLimit: Double, maxAirspeed: Double) {
        self.alphaLimit = alphaLimit
        self.maxAirspeed = maxAirspeed
    }
}

/// Which procedural cockpit the app builds around the pilot.
public enum CockpitStyle: String, Sendable {
    case spacecraftCapsule
    case fighterCanopy
    /// Bubble canopy with a wide view, for the passenger drone.
    case droneBubble
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
    public var world: WorldKind
    public var aerodynamics: AerodynamicModel?
    public var landingGear: LandingGear?
    public var aircraft: AircraftControl?
    public var multirotor: MultirotorControl?
    public var drag: BodyDrag?
    /// Fraction of max angular rate the attitude-hold loop may command (snappier for drones).
    public var holdRateFraction: Double
    public var startState: RigidBodyState
    public var course: Course
    /// Speedometer dial for this craft.
    public var speedGauge: GaugeScale
    /// Hand poses the pilot starts (and calibrates) in.
    public var startingPose: StartingPose

    public init(
        id: String, name: String, summary: String,
        mass: MassProperties, actuators: [Actuator],
        profile: ControlProfile, gains: ControllerGains,
        allocationWeights: ControlAllocator.Weights,
        cockpit: CockpitStyle, hullSize: Vec3,
        world: WorldKind = .deepSpace,
        aerodynamics: AerodynamicModel? = nil,
        landingGear: LandingGear? = nil,
        aircraft: AircraftControl? = nil,
        multirotor: MultirotorControl? = nil,
        drag: BodyDrag? = nil,
        holdRateFraction: Double = 0.25,
        startState: RigidBodyState = .init(),
        course: Course = CourseCatalog.asteroidSlalom,
        speedGauge: GaugeScale = .spacecraft,
        startingPose: StartingPose = .bothFlat
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
        self.world = world
        self.aerodynamics = aerodynamics
        self.landingGear = landingGear
        self.aircraft = aircraft
        self.multirotor = multirotor
        self.drag = drag
        self.holdRateFraction = holdRateFraction
        self.startState = startState
        self.course = course
        self.speedGauge = speedGauge
        self.startingPose = startingPose
    }

    /// The physics environment this craft is designed for.
    public var environment: any FlightEnvironment {
        switch world {
        case .deepSpace: DeepSpace()
        case .earth, .city: Atmosphere()
        }
    }

    /// Total forward thrust from main engines (N).
    public var maxForwardThrust: Double {
        actuators.filter { $0.kind == .mainEngine }
            .reduce(0) { $0 + max(0, -$1.direction.z) * $1.maxForce }
    }
}

public enum CraftCatalog {
    public static let all: [CraftDefinition] = [genericSpacecraft, jet, drone]
}
