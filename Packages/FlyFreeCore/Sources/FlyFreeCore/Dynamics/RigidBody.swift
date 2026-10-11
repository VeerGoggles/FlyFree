import simd

public struct MassProperties: Sendable, Equatable {
    /// kg
    public var mass: Double
    /// Principal moments of inertia about body X, Y, Z (kg·m²).
    public var inertia: Vec3

    public init(mass: Double, inertia: Vec3) {
        self.mass = mass
        self.inertia = inertia
    }

    /// Solid box of the given width (X), height (Y), length (Z).
    public static func box(mass: Double, width: Double, height: Double, length: Double) -> MassProperties {
        let k = mass / 12
        return MassProperties(mass: mass, inertia: Vec3(
            k * (height * height + length * length),
            k * (width * width + length * length),
            k * (width * width + height * height)
        ))
    }
}

public struct RigidBodyState: Sendable, Equatable {
    /// World position (m).
    public var position: Vec3
    /// World velocity (m/s).
    public var velocity: Vec3
    /// Body → world rotation.
    public var orientation: Quat
    /// Body-frame angular velocity (rad/s).
    public var angularVelocity: Vec3

    public init(
        position: Vec3 = .zero,
        velocity: Vec3 = .zero,
        orientation: Quat = Quat(ix: 0, iy: 0, iz: 0, r: 1),
        angularVelocity: Vec3 = .zero
    ) {
        self.position = position
        self.velocity = velocity
        self.orientation = orientation
        self.angularVelocity = angularVelocity
    }
}

public enum RigidBodyIntegrator {
    /// Semi-implicit Euler step with Euler's rotation equations.
    public static func step(
        _ s: inout RigidBodyState,
        mass: MassProperties,
        bodyWrench: Wrench,
        external: ExternalLoads,
        dt: Double
    ) {
        let i = mass.inertia
        let omega = s.angularVelocity
        let torque = bodyWrench.torque + external.bodyTorque
        let alpha = (torque - simd_cross(omega, i * omega)) / i
        s.angularVelocity += alpha * dt

        let w = s.angularVelocity
        let speed = simd_length(w)
        if speed > 1e-12 {
            s.orientation = (s.orientation * Quat(angle: speed * dt, axis: w / speed)).normalized
        }

        let accel = s.orientation.act(bodyWrench.force) / mass.mass + external.worldForce / mass.mass
        s.velocity += accel * dt
        s.position += s.velocity * dt
    }
}
