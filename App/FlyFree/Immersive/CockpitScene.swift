import FlyFreeCore
import RealityKit
import UIKit

/// Builds and animates the immersive scene. Everything is procedural so the project needs
/// no binary assets; swap in Reality Composer Pro cockpits per craft later.
@MainActor
final class CockpitScene {
    let root = Entity()
    /// Fixed around the pilot.
    let cockpit = Entity()
    /// Rotates (but never translates) with the craft: infinitely distant stars and the sun.
    private let sky = Entity()
    /// Rotates and translates with the craft: nearby objects for a sense of speed.
    private let world = Entity()
    private var thrusterIndicators: [ModelEntity] = []

    /// The point the universe pivots around — roughly a seated pilot's head.
    private let pivot = SIMD3<Float>(0, 1.2, 0)

    init(craft: CraftDefinition) {
        root.addChild(sky)
        root.addChild(world)
        root.addChild(cockpit)
        buildSky()
        buildWorld()
        buildCockpit(style: craft.cockpit)
        buildHologram(craft: craft)
    }

    func apply(_ telemetry: FlightTelemetry) {
        let s = telemetry.state
        let q = s.orientation
        let inverse = simd_quatf(ix: Float(-q.imag.x), iy: Float(-q.imag.y), iz: Float(-q.imag.z), r: Float(q.real))
        let position = SIMD3<Float>(Float(s.position.x), Float(s.position.y), Float(s.position.z))

        sky.orientation = inverse
        sky.position = pivot
        world.orientation = inverse
        world.position = pivot + inverse.act(-position)

        for (indicator, command) in zip(thrusterIndicators, telemetry.actuatorCommands) {
            indicator.scale = SIMD3(repeating: 0.4 + 2.2 * Float(command))
        }
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
            var p = SIMD3<Float>(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1) * 1500
            if simd_length(p) < 120 { p *= 4 }
            asteroid.position = p
            asteroid.scale = [1, 0.6 + rng.next() * 0.8, 0.7 + rng.next() * 0.6]
            world.addChild(asteroid)
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
