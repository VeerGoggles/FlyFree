import FlyFreeCore
import RealityKit
import UIKit

/// Base for a small hologram that floats above a hand, following it smoothly.
@MainActor
class HandHologram {
    let entity = Entity()
    /// Height above the hand's plane centroid (m), along world up.
    var lift: Float = 0.11
    private var position: SIMD3<Float>?

    func follow(_ plane: HandPlane?, visible: Bool) {
        guard visible, let plane else {
            entity.isEnabled = false
            position = nil
            return
        }
        entity.isEnabled = true
        let target = SIMD3<Float>(Float(plane.centroid.x), Float(plane.centroid.y) + lift, Float(plane.centroid.z))
        position = position.map { simd_mix($0, target, SIMD3(repeating: 0.35)) } ?? target
        entity.position = position!
    }
}

/// Above the stick hand: a miniature of the ship at its true attitude relative to the launch
/// frame, inside a level reference ring with a "launch forward" marker.
@MainActor
final class AttitudeHologram: HandHologram {
    private let ship = Entity()
    private let rateArrow: ModelEntity

    override init() {
        let cyan = MeshFactory.glow(.holoCyan, opacity: 0.9)
        let faint = MeshFactory.glow(.holoCyan, opacity: 0.25)
        rateArrow = ModelEntity(mesh: .generateBox(size: [0.002, 0.002, 1]), materials: [MeshFactory.glow(.holoAmber, opacity: 0.9)])
        super.init()

        // Level reference: ring + forward tick (fixed in the world/launch frame).
        entity.addChild(ModelEntity(mesh: MeshFactory.annulus(inner: 0.052, outer: 0.055), materials: [faint]))
        let forward = ModelEntity(mesh: MeshFactory.chevron(width: 0.012, length: 0.012), materials: [cyan])
        forward.position = [0, 0, -0.058]
        entity.addChild(forward)

        // Miniature ship (−Z forward).
        let hull = MeshFactory.glow(.holoCyan, opacity: 0.55)
        let edge = MeshFactory.glow(.white, opacity: 0.9)
        let fuselage = ModelEntity(mesh: .generateBox(size: [0.012, 0.008, 0.06], cornerRadius: 0.003), materials: [hull])
        let wings = ModelEntity(mesh: .generateBox(size: [0.06, 0.0015, 0.018], cornerRadius: 0.0007), materials: [hull])
        wings.position = [0, -0.001, 0.008]
        let fin = ModelEntity(mesh: .generateBox(size: [0.0015, 0.014, 0.012]), materials: [hull])
        fin.position = [0, 0.008, 0.022]
        let nose = ModelEntity(mesh: .generateSphere(radius: 0.0035), materials: [edge])
        nose.position = [0, 0, -0.031]
        let engine = ModelEntity(mesh: .generateSphere(radius: 0.004), materials: [MeshFactory.glow(.holoAmber)])
        engine.position = [0, 0, 0.031]
        for e in [fuselage, wings, fin, nose, engine] { ship.addChild(e) }
        entity.addChild(ship)
        entity.addChild(rateArrow)
    }

    func update(_ telemetry: FlightTelemetry?) {
        guard let telemetry else { return }
        let q = telemetry.state.orientation
        let attitude = simd_quatf(ix: Float(q.imag.x), iy: Float(q.imag.y), iz: Float(q.imag.z), r: Float(q.real))
        ship.orientation = attitude

        // Rotation-rate vector (world frame), drawn from the ship's center.
        let w = q.act(telemetry.state.angularVelocity)
        let rate = SIMD3<Float>(Float(w.x), Float(w.y), Float(w.z))
        let length = simd_length(rate) * 0.06
        rateArrow.isEnabled = length > 0.003
        if length > 0.003 {
            let dir = simd_normalize(rate)
            rateArrow.scale = [1, 1, length]
            rateArrow.orientation = simd_quatf(from: [0, 0, 1], to: dir)
            rateArrow.position = dir * length / 2
        }
    }
}

/// Above the throttle hand: the ship's thruster layout with each thruster's plume sized by its
/// command, plus a pedal gauge (centre = hold, up = accelerate, down = brake).
@MainActor
final class ThrusterHologram: HandHologram {
    private var plumes: [ModelEntity] = []
    private let pedalBar: ModelEntity
    private let brakeMaterial = MeshFactory.glow(.holoRed, opacity: 0.9)
    private let pedalMaterial = MeshFactory.glow(.holoGreen, opacity: 0.9)

    init(craft: CraftDefinition) {
        pedalBar = ModelEntity(mesh: .generateBox(size: [0.006, 1, 0.006]))
        super.init()
        let size = craft.hullSize
        let scale = 0.1 / Float(max(size.x, size.y, size.z))
        let model = Entity()
        model.scale = SIMD3(repeating: scale)
        // Three-quarter view so all thruster faces are visible.
        model.orientation = simd_quatf(angle: -0.5, axis: [0, 1, 0]) * simd_quatf(angle: 0.35, axis: [1, 0, 0])
        entity.addChild(model)

        model.addChild(ModelEntity(
            mesh: .generateBox(size: [Float(size.x) * 0.45, Float(size.y), Float(size.z)], cornerRadius: 0.6),
            materials: [MeshFactory.glow(.holoCyan, opacity: 0.18)]))

        for actuator in craft.actuators {
            let isMain = actuator.kind == .mainEngine
            let plume = ModelEntity(mesh: .generateSphere(radius: 1),
                                    materials: [MeshFactory.glow(isMain ? .holoAmber : UIColor(red: 0.85, green: 0.95, blue: 1, alpha: 1), opacity: 0.9)])
            // Plume sits on the exhaust side, opposite the force direction.
            let p = actuator.position - actuator.direction * 0.8
            plume.position = SIMD3(Float(p.x), Float(p.y), Float(p.z))
            let d = -actuator.direction
            plume.orientation = simd_quatf(from: [0, 0, 1], to: SIMD3(Float(d.x), Float(d.y), Float(d.z)))
            plume.components.set(PlumeComponent(isMain: isMain))
            model.addChild(plume)
            plumes.append(plume)
        }

        // Pedal gauge to the right of the ship.
        let track = ModelEntity(mesh: .generateBox(size: [0.002, 0.07, 0.002]), materials: [MeshFactory.glow(.holoCyan, opacity: 0.3)])
        track.position = [0.075, 0, 0]
        let zero = ModelEntity(mesh: .generateBox(size: [0.014, 0.0015, 0.0015]), materials: [MeshFactory.glow(.white, opacity: 0.8)])
        zero.position = [0.075, 0, 0]
        entity.addChild(track)
        entity.addChild(zero)
        entity.addChild(pedalBar)
    }

    func update(_ telemetry: FlightTelemetry?) {
        guard let telemetry else { return }
        for (plume, command) in zip(plumes, telemetry.actuatorCommands) {
            let isMain = plume.components[PlumeComponent.self]?.isMain ?? false
            let c = Float(command)
            let width: Float = isMain ? 0.6 : 0.3
            plume.isEnabled = c > 0.02
            plume.scale = [width * (0.6 + 0.4 * c), width * (0.6 + 0.4 * c), (isMain ? 2.5 : 1.4) * c]
        }
        let pedal = telemetry.input.throttleClosed ? -1 : Float(telemetry.input.throttle)
        let height = max(0.001, abs(pedal) * 0.035)
        pedalBar.scale = [1, height, 1]
        pedalBar.position = [0.075, pedal >= 0 ? height / 2 : -height / 2, 0]
        pedalBar.model?.materials = [pedal >= 0 ? pedalMaterial : brakeMaterial]
    }
}

struct PlumeComponent: Component {
    var isMain: Bool
}
