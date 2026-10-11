import Foundation
import simd

/// Spring-damper wheels with rolling, braking and cornering friction on flat ground.
public struct LandingGear: Sendable, Equatable {
    /// Wheel contact points in body axes (m), relative to the center of mass.
    public var wheels: [Vec3]
    public var stiffness: Double      // N/m per wheel
    public var damping: Double        // N·s/m per wheel
    public var rollingFriction = 0.02
    public var brakingFriction = 0.55
    public var corneringFriction = 0.8

    public init(wheels: [Vec3], stiffness: Double, damping: Double) {
        self.wheels = wheels
        self.stiffness = stiffness
        self.damping = damping
    }

    public struct Result: Sendable {
        public var loads: ExternalLoads
        public var wheelsOnGround: Int
    }

    public func loads(state: RigidBodyState, groundHeight: Double, brakes: Bool) -> Result {
        var total = ExternalLoads.none
        var contacts = 0
        let q = state.orientation
        let omegaWorld = q.act(state.angularVelocity)
        var forward = q.act(BodyAxis.forward)
        forward.y = 0
        forward = simd_length(forward) > 1e-6 ? simd_normalize(forward) : Vec3(0, 0, -1)
        let lateral = simd_cross(forward, Vec3(0, 1, 0))

        for wheel in wheels {
            let r = q.act(wheel)
            let p = state.position + r
            let depth = groundHeight - p.y
            guard depth > 0 else { continue }
            contacts += 1
            let v = state.velocity + simd_cross(omegaWorld, r)
            let normal = max(0, stiffness * depth - damping * v.y)
            let along = simd_dot(v, forward), across = simd_dot(v, lateral)
            let mu = brakes ? brakingFriction : rollingFriction
            let friction = -forward * (mu * normal * tanh(along / 0.3))
                - lateral * (corneringFriction * normal * tanh(across / 0.3))
            let f = Vec3(0, normal, 0) + friction
            total.worldForce += f
            total.bodyTorque += q.inverse.act(simd_cross(r, f))
        }
        return Result(loads: total, wheelsOnGround: contacts)
    }
}
