import simd

/// A ring to fly through.
public struct Gate: Sendable, Equatable, Identifiable {
    public var id: Int
    /// World position of the ring's center (m).
    public var center: Vec3
    /// Unit direction you must be travelling to count as flying through.
    public var direction: Vec3
    /// Inner radius of the ring (m); the craft's center must pass inside it.
    public var radius: Double

    public init(id: Int, center: Vec3, direction: Vec3 = BodyAxis.forward, radius: Double) {
        self.id = id
        self.center = center
        self.direction = simd_normalize(direction)
        self.radius = radius
    }
}

/// An ordered set of gates.
public struct Course: Sendable, Equatable {
    public var name: String
    public var gates: [Gate]

    public init(name: String, gates: [Gate]) {
        self.name = name
        self.gates = gates
    }
}

public enum CourseCatalog {
    /// Nine airborne gates looping out from the runway and back, for the jet. Each gate faces
    /// along the route from the previous one.
    public static let valleyRun: Course = {
        let centers: [Vec3] = [
            Vec3(0, 120, -3500), Vec3(300, 220, -6000), Vec3(1200, 300, -8200),
            Vec3(2600, 350, -9500), Vec3(4200, 300, -9800), Vec3(5600, 250, -8800),
            Vec3(6400, 200, -7000), Vec3(6500, 180, -4800), Vec3(5800, 160, -2800),
        ]
        var previous = Vec3(0, 0, 0)
        var gates: [Gate] = []
        for (i, center) in centers.enumerated() {
            gates.append(Gate(id: i, center: center, direction: center - previous, radius: 45))
            previous = center
        }
        return Course(name: "Valley Run", gates: gates)
    }()

    /// Twelve gates weaving ahead of the launch point through the asteroid field.
    public static let asteroidSlalom = Course(
        name: "Asteroid Slalom",
        gates: (1...12).map { i in
            Gate(id: i - 1,
                 center: Vec3(Double(i % 3 - 1) * 40, Double(i % 2) * 30 - 15, Double(-250 * i)),
                 radius: 25)
        }
    )
}
