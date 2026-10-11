extension CraftCatalog {
    /// A generic 12 t spacecraft: two main engines at the tail, a 14-thruster RCS
    /// (pitch/yaw/strafe quads at nose and tail, roll pairs on the wingtips, two retro
    /// thrusters at the nose). Thrusters are sized for strong braking. No gravity or atmosphere is assumed.
    public static let genericSpacecraft: CraftDefinition = {
        let width = 6.0, height = 3.0, length = 14.0
        let nose = -length / 2 + 1, tail = length / 2 - 1
        // Sized so the craft can reach and stop rotations quickly (up to ~2.5 rad/s² in pitch/yaw)
        // and brake about 3× harder than it speeds up.
        let rcsForce = 50_000.0
        let retroForce = 75_000.0

        var actuators: [Actuator] = [
            Actuator(id: "main.L", kind: .mainEngine, position: Vec3(-1.5, 0, length / 2),
                     direction: BodyAxis.forward, maxForce: 60_000, priority: 0),
            Actuator(id: "main.R", kind: .mainEngine, position: Vec3(1.5, 0, length / 2),
                     direction: BodyAxis.forward, maxForce: 60_000, priority: 0),
        ]

        // Pitch / yaw / strafe quads: thrusters on each face push inward.
        for (station, z) in [("nose", nose), ("tail", tail)] {
            let faces: [(String, Vec3, Vec3)] = [
                ("up", Vec3(0, height / 2, z), Vec3(0, -1, 0)),
                ("down", Vec3(0, -height / 2, z), Vec3(0, 1, 0)),
                ("left", Vec3(-1, 0, z), Vec3(1, 0, 0)),
                ("right", Vec3(1, 0, z), Vec3(-1, 0, 0)),
            ]
            for (face, position, direction) in faces {
                actuators.append(Actuator(id: "rcs.\(station).\(face)", kind: .rcsThruster,
                                          position: position, direction: direction,
                                          maxForce: rcsForce, priority: 1))
            }
        }

        // Roll pairs on the wingtips.
        for (side, x) in [("L", -width / 2), ("R", width / 2)] {
            actuators.append(Actuator(id: "rcs.wing\(side).up", kind: .rcsThruster,
                                      position: Vec3(x, 0, 0), direction: Vec3(0, 1, 0),
                                      maxForce: rcsForce, priority: 1))
            actuators.append(Actuator(id: "rcs.wing\(side).down", kind: .rcsThruster,
                                      position: Vec3(x, 0, 0), direction: Vec3(0, -1, 0),
                                      maxForce: rcsForce, priority: 1))
        }

        // Retro thrusters for braking without flipping.
        for (side, x) in [("L", -1.0), ("R", 1.0)] {
            actuators.append(Actuator(id: "rcs.retro\(side)", kind: .rcsThruster,
                                      position: Vec3(x, 0, -length / 2), direction: Vec3(0, 0, 1),
                                      maxForce: retroForce, priority: 1))
        }

        return CraftDefinition(
            id: "generic-spacecraft",
            name: "Generic Spacecraft",
            summary: "A forgiving 12-tonne test craft. Deep space: no gravity, no drag, no wind.",
            mass: .box(mass: 12_000, width: width, height: height, length: length),
            actuators: actuators,
            profile: ControlProfile(
                maxAngularRate: Vec3(0.6, 0.5, 1.0),
                // Rotation and translation are tuned independently.
                angularAcceleration: Vec3(1.2, 1.2, 1.8),   // rad/s² spin-up   (max rate in ~0.5 s)
                angularDeceleration: Vec3(2.4, 2.4, 3.6),   // rad/s² spin-down (stops in ~0.25 s)
                linearAcceleration: 4,                       // m/s² (was 10)
                linearDeceleration: 12,                      // m/s² braking
                maxForwardSpeed: 120_000,
                maxStrafeSpeed: 10
            ),
            gains: ControllerGains(),
            // Torque errors weighted per meter of lever arm so N and N·m are comparable.
            allocationWeights: .init(force: 1, torque: 1.0 / 5.0),
            cockpit: .spacecraftCapsule,
            hullSize: Vec3(width, height, length)
        )
    }()
}
