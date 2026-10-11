extension CraftCatalog {
    /// A two-seat passenger quadcopter (air-taxi class): 450 kg, four 2.5 kN rotors on 2.2 m arms,
    /// bubble canopy. Flown like a camera drone: tilt to move, twist to yaw, pedal to climb, let go
    /// to hover. Starts on a helipad in the city.
    public static let drone: CraftDefinition = {
        let arm = 2.2
        let rotors: [Actuator] = [
            .rotor(id: "rotor.frontLeft", position: Vec3(-arm, 0.6, -arm), maxThrust: 2_500, spin: 1, torqueRatio: 0.15),
            .rotor(id: "rotor.frontRight", position: Vec3(arm, 0.6, -arm), maxThrust: 2_500, spin: -1, torqueRatio: 0.15),
            .rotor(id: "rotor.rearRight", position: Vec3(arm, 0.6, arm), maxThrust: 2_500, spin: 1, torqueRatio: 0.15),
            .rotor(id: "rotor.rearLeft", position: Vec3(-arm, 0.6, arm), maxThrust: 2_500, spin: -1, torqueRatio: 0.15),
        ]
        let skidHeight = 1.3
        let skids = LandingGear(wheels: [Vec3(-0.8, -skidHeight, -0.9), Vec3(0.8, -skidHeight, -0.9),
                                         Vec3(0.8, -skidHeight, 0.9), Vec3(-0.8, -skidHeight, 0.9)],
                                stiffness: 40_000, damping: 6_000)
        var gear = skids
        gear.rollingFriction = 0.6  // skids, not wheels
        gear.brakingFriction = 0.8

        return CraftDefinition(
            id: "drone",
            name: "Passenger Drone",
            summary: "A two-seat air-taxi quadcopter. Tilt to fly, twist to turn, pedal to climb; let go to hover. Thread the gates between the towers.",
            mass: MassProperties(mass: 450, inertia: Vec3(520, 500, 520)),
            actuators: rotors,
            profile: ControlProfile(
                maxAngularRate: Vec3(1.5, 1.2, 1.5),
                angularAcceleration: Vec3(4, 1.0, 4),
                angularDeceleration: Vec3(6, 1.2, 6),
                linearAcceleration: 4,     // velocity reference ramp (m/s²)
                linearDeceleration: 6,
                maxForwardSpeed: 18,
                maxStrafeSpeed: 18
            ),
            gains: ControllerGains(rate: Vec3(8, 6, 8), attitude: 5, velocity: 1.6),
            allocationWeights: .init(force: 1, torque: 1.0 / 2.0),
            cockpit: .droneBubble,
            hullSize: Vec3(2 * arm + 2, 2.2, 2 * arm + 2),
            world: .city,
            landingGear: gear,
            multirotor: MultirotorControl(maxHorizontalSpeed: 18, maxClimbRate: 6, maxDescentRate: 4,
                                          maxYawRate: 0.9, maxTilt: 0.45),
            drag: BodyDrag(dragArea: 1.4, angularDamping: Vec3(150, 60, 150)),
            holdRateFraction: 1,
            startState: RigidBodyState(position: Vec3(0, skidHeight - 0.05, 0)),
            course: CourseCatalog.rooftopRun,
            speedGauge: GaugeScale(kind: .linear(max: 40, major: 10, minor: 2), unit: "m/s"),
            startingPose: .controllerGrip
        )
    }()
}
