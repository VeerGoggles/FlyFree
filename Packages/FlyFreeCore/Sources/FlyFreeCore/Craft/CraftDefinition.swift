/// Which world the craft flies in (physics environment and scenery).
public enum WorldKind: String, Sendable {
    case deepSpace
    case earth
    /// Earth physics with a city to fly through.
    case city
}

/// Flight computer for multirotors, flown like a radio-controlled drone in **angle mode** with a
/// **raw collective**:
/// - throttle-hand height → total rotor thrust (stick centre = hover thrust, as in PX4's
///   rescaled throttle curve); no altitude hold;
/// - stick-hand tilt → target pitch/roll angle (self-levels when the hand is flat), as in
///   Betaflight / PX4 angle mode; stick-hand twist → yaw rate.
public struct MultirotorControl: Sendable, Equatable {
    /// Maximum tilt from level (rad), like Betaflight's `angle_limit`.
    public var maxTilt: Double
    /// Drone tilt per radian of hand tilt (1 = the drone copies the hand).
    public var tiltPerHandTilt: Double
    public var maxYawRate: Double
    /// Highest collective the throttle hand can command, as a fraction of the rotors' total
    /// thrust; the rest is kept for attitude control (like a mixer's desaturation headroom).
    public var maxCollectiveFraction = 0.85
    /// Collective (fraction of hover) while sitting on the ground with the hand at or below centre.
    public var groundIdle = 0.3
    /// Closed throttle hand: vertical-speed gain (1/s) of the hover assist.
    public var hoverAssistGain = 1.5
    /// Hand tilt dead zone and response (normalized like `PilotInput.rotation`).
    public var tiltShaping = AxisShaping(deadzone: 0.03, expo: 0)

    public init(maxTilt: Double, tiltPerHandTilt: Double, maxYawRate: Double) {
        self.maxTilt = maxTilt
        self.tiltPerHandTilt = tiltPerHandTilt
        self.maxYawRate = maxYawRate
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
    /// What the throttle hand controls for this craft.
    public var throttleHand: ThrottleHandMode

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
        startingPose: StartingPose = .bothFlat,
        throttleHand: ThrottleHandMode = .pedal
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
        self.throttleHand = throttleHand
    }

    /// The physics environment this craft is designed for.
    public var environment: any FlightEnvironment {
        switch world {
        case .deepSpace: DeepSpace()
        case .earth, .city: Atmosphere()
        }
    }

    /// Total upward thrust from rotors (N).
    public var maxRotorThrust: Double {
        actuators.filter { $0.kind == .rotor }.reduce(0) { $0 + max(0, $1.direction.y) * $1.maxForce }
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
