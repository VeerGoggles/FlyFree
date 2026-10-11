import FlyFreeCore
import RealityKit
import UIKit

/// Builds and animates the immersive scene. Everything is procedural so the project needs
/// no binary assets; swap in Reality Composer Pro cockpits per craft later.
@MainActor
final class CockpitScene {
    let root = Entity()
    /// Session guidance (calibration panel, hand planes). Never faded.
    private let guide = Entity()
    /// Cockpit + universe. Fades in after the first calibration.
    private let flight = Entity()
    /// Fixed around the pilot.
    let cockpit = Entity()
    /// Rotates (but never translates) with the craft: infinitely distant stars and the sun.
    private let sky = Entity()
    /// Rotates and translates with the craft: the gate course.
    private let world = Entity()
    /// Rotates with the craft; its asteroids wrap around the craft in a repeating cube, so
    /// there is always something streaming past, even at tens of km/s.
    private let field = Entity()
    private var asteroids: [(entity: ModelEntity, base: Vec3)] = []
    private let fieldSize = 3000.0
    private var thrusterIndicators: [ModelEntity] = []
    private var handPaddles: [FlyFreeCore.Chirality: ModelEntity] = [:]
    private var paddleStyles: [FlyFreeCore.Chirality: PaddleStyle] = [:]

    /// The point the universe pivots around — roughly a seated pilot's head.
    private let pivot = SIMD3<Float>(0, 1.2, 0)

    init(craft: CraftDefinition) {
        root.addChild(flight)
        root.addChild(guide)
        flight.components.set(OpacityComponent(opacity: 0))
        flight.addChild(sky)
        flight.addChild(world)
        flight.addChild(field)
        flight.addChild(cockpit)
        buildBackdrop()
        buildSky()
        buildWorld()
        buildCockpit(style: craft.cockpit)
        buildHologram(craft: craft)
        buildHandPaddles()
    }

    func mountInCockpit(_ entity: Entity?, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        mount(entity, in: cockpit, position: position, yaw: yaw, pitch: pitch)
    }

    func mountInGuide(_ entity: Entity?, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        mount(entity, in: guide, position: position, yaw: yaw, pitch: pitch)
    }

    private func mount(_ entity: Entity?, in parent: Entity, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        guard let entity else { return }
        entity.position = position
        entity.orientation = simd_quatf(angle: yaw, axis: [0, 1, 0]) * simd_quatf(angle: pitch, axis: [1, 0, 0])
        parent.addChild(entity)
    }

    func apply(_ model: AppModel) {
        flight.components.set(OpacityComponent(opacity: model.sceneOpacity))
        updateHandPaddles(model)
        guard let telemetry = model.telemetry else { return }

        let s = telemetry.state
        let q = s.orientation
        let inverse = simd_quatf(ix: Float(-q.imag.x), iy: Float(-q.imag.y), iz: Float(-q.imag.z), r: Float(q.real))
        let position = SIMD3<Float>(Float(s.position.x), Float(s.position.y), Float(s.position.z))

        sky.orientation = inverse
        sky.position = pivot
        world.orientation = inverse
        world.position = pivot + inverse.act(-position)
        field.orientation = inverse
        field.position = pivot

        // Wrap in Double so positions stay precise however far the craft has flown.
        for (entity, base) in asteroids {
            var offset = base - s.position
            offset -= fieldSize * ((offset + fieldSize / 2) / fieldSize).rounded(.down)
            entity.position = SIMD3(Float(offset.x), Float(offset.y), Float(offset.z))
        }

        for (indicator, command) in zip(thrusterIndicators, telemetry.actuatorCommands) {
            indicator.scale = SIMD3(repeating: 0.4 + 2.2 * Float(command))
        }
    }

    // MARK: - Hand planes

    private enum PaddleStyle: Equatable {
        case waiting, ready, engaged, released, resetting, braking
        var color: UIColor {
            switch self {
            case .waiting: UIColor(red: 1, green: 0.6, blue: 0.1, alpha: 1)
            case .ready, .engaged: UIColor(red: 0.2, green: 1, blue: 0.5, alpha: 1)
            case .released: UIColor(white: 0.6, alpha: 1)
            case .resetting: UIColor(red: 0.7, green: 0.4, blue: 1, alpha: 1)
            case .braking: UIColor(red: 1, green: 0.25, blue: 0.2, alpha: 1)
            }
        }
    }

    /// A thin translucent plate on each hand's fitted plane — the "stick" the pilot is holding.
    private func buildHandPaddles() {
        for chirality in FlyFreeCore.Chirality.allCases {
            let paddle = ModelEntity(mesh: .generateBox(size: [0.085, 0.003, 0.11], cornerRadius: 0.0015))
            paddle.isEnabled = false
            guide.addChild(paddle)
            handPaddles[chirality] = paddle
        }
    }

    private func updateHandPaddles(_ model: AppModel) {
        for (chirality, paddle) in handPaddles {
            guard let plane = model.handPlanes[chirality] else {
                paddle.isEnabled = false
                continue
            }
            paddle.isEnabled = true
            // Float the plate just above the back of the hand.
            let c = plane.centroid + plane.normal * 0.025
            let q = plane.orientation
            paddle.position = SIMD3(Float(c.x), Float(c.y), Float(c.z))
            paddle.orientation = simd_quatf(ix: Float(q.imag.x), iy: Float(q.imag.y), iz: Float(q.imag.z), r: Float(q.real))

            let style: PaddleStyle
            if case .calibrating = model.phase {
                let readiness = chirality == .left ? model.calibrationProgress.left : model.calibrationProgress.right
                style = readiness == .ready ? .ready : .waiting
            } else if chirality == model.dominantHand {
                let input = model.telemetry?.input
                style = input?.stickClosed == true ? .resetting : (input?.engaged == true ? .engaged : .released)
            } else {
                style = model.telemetry?.input.throttleClosed == true ? .braking : .engaged
            }
            if paddleStyles[chirality] != style {
                paddleStyles[chirality] = style
                var material = UnlitMaterial(color: style.color)
                material.blending = .transparent(opacity: 0.45)
                paddle.model?.materials = [material]
            }
        }
    }

    // MARK: - Backdrop

    /// An opaque inside-out sphere that hides the room once the scene has faded in.
    private func buildBackdrop() {
        let backdrop = ModelEntity(mesh: .generateSphere(radius: 4000),
                                   materials: [UnlitMaterial(color: UIColor(red: 0.005, green: 0.005, blue: 0.02, alpha: 1))])
        backdrop.scale = [-1, 1, 1]  // flip winding so the inside faces are rendered
        backdrop.position = pivot
        flight.addChild(backdrop)
    }

    // MARK: - Sky

    private func buildSky() {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        var rng = SeededRandom(seed: 144)
        for _ in 0..<2500 {
            let dir = simd_normalize(SIMD3<Float>(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1) + 1e-4)
            let center = dir * 900
            let size = 0.8 + rng.next() * 2.2
            let base = UInt32(positions.count)
            positions += [
                center + SIMD3(size, 0, 0), center + SIMD3(-size, 0, 0),
                center + SIMD3(0, size, 0), center + SIMD3(0, -size, 0),
                center + SIMD3(0, 0, size), center + SIMD3(0, 0, -size),
            ]
            // Octahedron: visible from any direction.
            let faces: [UInt32] = [0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5]
            indices += faces.map { base + $0 }
        }
        var descriptor = MeshDescriptor(name: "stars")
        descriptor.positions = MeshBuffer(positions)
        descriptor.primitives = .triangles(indices)
        if let mesh = try? MeshResource.generate(from: [descriptor]) {
            sky.addChild(ModelEntity(mesh: mesh, materials: [UnlitMaterial(color: .white)]))
        }

        let sun = DirectionalLight()
        sun.light.intensity = 3000
        sun.look(at: .zero, from: [400, 300, 200], relativeTo: nil)
        sky.addChild(sun)
    }

    // MARK: - World

    private func buildWorld() {
        var rng = SeededRandom(seed: 2026)
        let rock = SimpleMaterial(color: UIColor(white: 0.45, alpha: 1), roughness: 0.9, isMetallic: false)
        for _ in 0..<160 {
            let radius = 3 + rng.next() * 25
            let asteroid = ModelEntity(mesh: .generateSphere(radius: radius), materials: [rock])
            var p = Vec3(Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1) * fieldSize / 2
            if simd_length(p) < 120 { p *= 4 }
            asteroid.scale = [1, 0.6 + rng.next() * 0.8, 0.7 + rng.next() * 0.6]
            field.addChild(asteroid)
            asteroids.append((asteroid, p))
        }

        // A line of gates straight ahead to fly through.
        let gateMaterial = UnlitMaterial(color: UIColor(red: 0.2, green: 0.9, blue: 1, alpha: 1))
        for i in 1...12 {
            let gate = Entity()
            let z = Float(-250 * i)
            let segments = 24
            for k in 0..<segments {
                let a = Float(k) / Float(segments) * 2 * .pi
                let bar = ModelEntity(mesh: .generateBox(width: 6.6, height: 0.6, depth: 0.6), materials: [gateMaterial])
                bar.position = [cos(a) * 25, sin(a) * 25, 0]
                bar.orientation = simd_quatf(angle: a + .pi / 2, axis: [0, 0, 1])
                gate.addChild(bar)
            }
            gate.position = [Float(i % 3 - 1) * 40, Float(i % 2) * 30 - 15, z]
            world.addChild(gate)
        }
    }

    // MARK: - Cockpit

    private func buildCockpit(style: CockpitStyle) {
        let hull = SimpleMaterial(color: UIColor(white: 0.18, alpha: 1), roughness: 0.6, isMetallic: true)
        let trim = SimpleMaterial(color: UIColor(white: 0.32, alpha: 1), roughness: 0.4, isMetallic: true)
        let glow = UnlitMaterial(color: UIColor(red: 1, green: 0.55, blue: 0.1, alpha: 1))

        func box(_ size: SIMD3<Float>, _ position: SIMD3<Float>, _ material: any RealityKit.Material, tilt: Float = 0) {
            let e = ModelEntity(mesh: .generateBox(size: size, cornerRadius: 0.01), materials: [material])
            e.position = position
            e.orientation = simd_quatf(angle: tilt, axis: [1, 0, 0])
            cockpit.addChild(e)
        }

        func strut(from a: SIMD3<Float>, to b: SIMD3<Float>, thickness: Float = 0.05) {
            let length = simd_length(b - a)
            let e = ModelEntity(mesh: .generateBox(size: [thickness, length, thickness]), materials: [trim])
            e.position = (a + b) / 2
            e.orientation = simd_quatf(from: [0, 1, 0], to: simd_normalize(b - a))
            cockpit.addChild(e)
        }

        // Floor, main console, side consoles.
        box([1.8, 0.03, 2.0], [0, 0, -0.2], hull)
        box([1.3, 0.05, 0.4], [0, 0.82, -0.6], hull, tilt: 0.35)
        box([1.3, 0.8, 0.05], [0, 0.42, -0.78], hull)
        for x: Float in [-0.68, 0.68] {
            box([0.3, 0.06, 0.8], [x, 0.7, -0.15], hull)
            box([0.3, 0.7, 0.05], [x, 0.35, -0.55], hull)
        }
        // Accent light strip.
        box([1.2, 0.01, 0.01], [0, 0.86, -0.42], glow)

        // Canopy frame.
        let height: Float = style == .fighterCanopy ? 1.75 : 2.05
        let sill: [SIMD3<Float>] = [[-0.8, 0.9, -0.8], [0.8, 0.9, -0.8], [0.9, 0.9, 0.4], [-0.9, 0.9, 0.4]]
        for i in 0..<sill.count { strut(from: sill[i], to: sill[(i + 1) % sill.count]) }
        strut(from: [-0.8, 0.9, -0.8], to: [-0.45, height, -0.35])
        strut(from: [0.8, 0.9, -0.8], to: [0.45, height, -0.35])
        strut(from: [-0.45, height, -0.35], to: [0.45, height, -0.35])
        strut(from: [-0.45, height, -0.35], to: [-0.6, height, 0.5])
        strut(from: [0.45, height, -0.35], to: [0.6, height, 0.5])
        strut(from: [-0.9, 0.9, 0.4], to: [-0.6, height, 0.5])
        strut(from: [0.9, 0.9, 0.4], to: [0.6, height, 0.5])

        // Interior fill light so the cockpit is readable in deep space.
        let fill = PointLight()
        fill.light.intensity = 400
        fill.light.attenuationRadius = 3
        fill.position = [0, 1.9, 0.2]
        cockpit.addChild(fill)
    }

    /// A small hologram of the craft on the console, with a dot per actuator that grows
    /// with its command — a live view of what the allocator is doing.
    private func buildHologram(craft: CraftDefinition) {
        let hologram = Entity()
        let size = craft.hullSize
        let scale = 0.22 / Float(max(size.x, size.y, size.z))
        hologram.position = [0.38, 0.97, -0.55]
        hologram.orientation = simd_quatf(angle: -0.6, axis: [0, 1, 0])
        hologram.scale = SIMD3(repeating: scale)
        cockpit.addChild(hologram)

        var bodyMaterial = UnlitMaterial(color: UIColor(red: 0.3, green: 0.8, blue: 1, alpha: 1))
        bodyMaterial.blending = .transparent(opacity: 0.25)
        hologram.addChild(ModelEntity(
            mesh: .generateBox(size: [Float(size.x) * 0.5, Float(size.y), Float(size.z)], cornerRadius: 0.5),
            materials: [bodyMaterial]
        ))

        let engineMaterial = UnlitMaterial(color: UIColor(red: 1, green: 0.5, blue: 0.1, alpha: 1))
        let rcsMaterial = UnlitMaterial(color: UIColor(red: 1, green: 1, blue: 0.6, alpha: 1))
        for actuator in craft.actuators {
            let isMain = actuator.kind == .mainEngine
            let dot = ModelEntity(mesh: .generateSphere(radius: isMain ? 0.7 : 0.35),
                                  materials: [isMain ? engineMaterial : rcsMaterial])
            // Plume sits on the exhaust side, opposite the force direction.
            let p = actuator.position - actuator.direction * 0.6
            dot.position = SIMD3(Float(p.x), Float(p.y), Float(p.z))
            hologram.addChild(dot)
            thrusterIndicators.append(dot)
        }
    }
}

/// Small deterministic RNG so the starfield and asteroids are the same every launch.
struct SeededRandom {
    private var state: UInt64
    init(seed: UInt64) { state = seed &* 6364136223846793005 &+ 1442695040888963407 }
    mutating func next() -> Float {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return Float(state >> 40) / Float(1 << 24)
    }
}
