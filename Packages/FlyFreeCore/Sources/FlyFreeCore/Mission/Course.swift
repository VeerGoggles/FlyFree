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
