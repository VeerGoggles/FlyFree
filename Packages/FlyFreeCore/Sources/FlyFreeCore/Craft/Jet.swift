import Foundation

extension CraftCatalog {
    /// A light single-engine fighter (roughly F-16 class): 12 t, 100 kN, fly-by-wire.
    /// Earth-like world with gravity, atmosphere and a runway; starts lined up for take-off.
    public static let jet: CraftDefinition = {
        let aero = AerodynamicModel(
            wingArea: 27.9, span: 9.45, chord: 3.45,
            cl0: 0.1, clAlpha: 4.5, stallAlpha: 0.30,
            cd0: 0.021, inducedDragFactor: 0.12, speedBrakeDrag: 0.08, sideForceBeta: 0.9,
            cm0: 0, cmAlpha: -0.6, cmPitchDamping: -8,
            rollDihedral: 0.08, rollDamping: -0.4,
            yawStability: 0.12, yawDamping: -0.25)
        let s = aero.wingArea, b = aero.span, c = aero.chord

        let actuators: [Actuator] = [
            Actuator(id: "engine", kind: .mainEngine, position: Vec3(0, 0, 7.5),
                     direction: BodyAxis.forward, maxForce: 100_000, priority: 0),
            // Control surfaces as pure moments per pascal of dynamic pressure at full deflection.
            .surface(id: "elevator", momentPerPascal: Vec3(s * c * 0.3, 0, 0), position: Vec3(0, 0.3, 6.5),
                     visual: .init(hinge: BodyAxis.right, span: 5.5)),
            .surface(id: "ailerons", momentPerPascal: Vec3(0, 0, s * b * 0.03), position: Vec3(3.6, 0, 1.5),
                     visual: .init(hinge: BodyAxis.right, span: 2.0, mirrored: true)),
            .surface(id: "rudder", momentPerPascal: Vec3(0, s * b * 0.05, 0), position: Vec3(0, 2.2, 6.2),
                     visual: .init(hinge: BodyAxis.up, span: 2.2)),
        ]

        let gearHeight = 1.6
        let gear = LandingGear(wheels: [Vec3(0, -gearHeight, -4.5), Vec3(-1.3, -gearHeight, 0.8), Vec3(1.3, -gearHeight, 0.8)],
                               stiffness: 300_000, damping: 42_000)

        return CraftDefinition(
            id: "jet",
            name: "Atmospheric Jet",
            summary: "A light fly-by-wire fighter. Gravity, lift and drag; take off from the runway and fly the valley gates.",
            mass: MassProperties(mass: 12_000, inertia: Vec3(75_673, 85_552, 12_875)),
            actuators: actuators,
            profile: ControlProfile(
                maxAngularRate: Vec3(0.5, 0.25, 2.5),     // pitch / yaw / roll (rad/s)
                angularAcceleration: Vec3(2.0, 1.0, 6.0),
                angularDeceleration: Vec3(4.0, 2.0, 10.0),
                linearAcceleration: 6,                   // autothrottle target ramp (m/s²)
                linearDeceleration: 8,
                levelingGain: 1.5,
                maxForwardSpeed: 300,
                maxStrafeSpeed: 0
            ),
            gains: ControllerGains(rate: Vec3(5, 4, 6), attitude: 1.5, velocity: 1.5),
            allocationWeights: .init(force: 1, torque: 1.0 / 5.0),
            cockpit: .fighterCanopy,
            hullSize: Vec3(9.45, 4.8, 15),
            world: .earth,
            aerodynamics: aero,
            landingGear: gear,
            aircraft: AircraftControl(alphaLimit: 0.24, maxAirspeed: 300),
            startState: RigidBodyState(position: Vec3(0, gearHeight - 0.12, 0)),
            course: CourseCatalog.valleyRun,
            speedGauge: .jetAirspeed,
            startingPose: .bothFlat
        )
    }()
}
