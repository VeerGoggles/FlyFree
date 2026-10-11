import RealityKit
import UIKit

/// Procedural meshes and materials — FlyFree ships no binary assets.
@MainActor
enum MeshFactory {
    /// Flat ring in the XZ plane, visible from both sides.
    static func annulus(inner: Float, outer: Float, segments: Int = 64) -> MeshResource {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        for i in 0...segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            let d = SIMD3<Float>(cos(a), 0, sin(a))
            positions += [d * inner, d * outer]
        }
        for i in 0..<UInt32(segments) {
            let a = i * 2, b = a + 1, c = a + 2, d = a + 3
            indices += [a, c, b, b, c, d, a, b, c, b, d, c]
        }
        return mesh("annulus", positions, indices)
    }

    /// Flat disc in the XZ plane, visible from both sides.
    static func disc(radius: Float, segments: Int = 48) -> MeshResource {
        annulus(inner: 0.0001, outer: radius, segments: segments)
    }

    /// Torus around the Z axis (a ring you fly through along Z).
    static func torus(major: Float, minor: Float, segments: Int = 72, sides: Int = 12) -> MeshResource {
        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        for i in 0...segments {
            let u = Float(i) / Float(segments) * 2 * .pi
            let center = SIMD3<Float>(cos(u), sin(u), 0) * major
            for j in 0...sides {
                let v = Float(j) / Float(sides) * 2 * .pi
                let n = SIMD3<Float>(cos(u) * cos(v), sin(u) * cos(v), sin(v))
                positions.append(center + n * minor)
                normals.append(n)
            }
        }
        let stride = UInt32(sides + 1)
        for i in 0..<UInt32(segments) {
            for j in 0..<UInt32(sides) {
                let a = i * stride + j, b = a + stride
                indices += [a, b, a + 1, a + 1, b, b + 1]
            }
        }
        var d = MeshDescriptor(name: "torus")
        d.positions = MeshBuffer(positions)
        d.normals = MeshBuffer(normals)
        d.primitives = .triangles(indices)
        return try! MeshResource.generate(from: [d])
    }

    /// Vertical cylindrical wall (no caps), visible from both sides — horizon haze.
    static func band(radius: Float, height: Float, segments: Int = 96) -> MeshResource {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        for i in 0...segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            positions += [SIMD3(cos(a) * radius, 0, sin(a) * radius), SIMD3(cos(a) * radius, height, sin(a) * radius)]
        }
        for i in 0..<UInt32(segments) {
            let a = i * 2, b = a + 1, c = a + 2, d = a + 3
            indices += [a, c, b, b, c, d, a, b, c, b, d, c]
        }
        return mesh("band", positions, indices)
    }

    /// Flat arrowhead in the XZ plane pointing -Z, both sides.
    static func chevron(width: Float, length: Float) -> MeshResource {
        let p: [SIMD3<Float>] = [[0, 0, -length], [-width / 2, 0, 0], [0, 0, -length * 0.35], [width / 2, 0, 0]]
        return mesh("chevron", p, [0, 1, 2, 0, 2, 1, 0, 2, 3, 0, 3, 2])
    }

    /// Many small octahedra in one mesh (stars, dust).
    static func points(_ centers: [SIMD3<Float>], size: (Int) -> Float) -> MeshResource {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        let faces: [UInt32] = [0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5]
        for (i, c) in centers.enumerated() {
            let s = size(i)
            let base = UInt32(positions.count)
            positions += [c + [s, 0, 0], c - [s, 0, 0], c + [0, s, 0], c - [0, s, 0], c + [0, 0, s], c - [0, 0, s]]
            indices += faces.map { base + $0 }
        }
        return mesh("points", positions, indices)
    }

    static func glow(_ color: UIColor, opacity: Float = 1) -> UnlitMaterial {
        var m = UnlitMaterial(color: color)
        if opacity < 1 { m.blending = .transparent(opacity: .init(floatLiteral: opacity)) }
        return m
    }

    static func metal(_ white: CGFloat, roughness: Float = 0.45) -> PhysicallyBasedMaterial {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: UIColor(white: white, alpha: 1))
        m.metallic = 0.85
        m.roughness = .init(floatLiteral: roughness)
        return m
    }

    static func emissive(_ color: UIColor, intensity: Float = 1.5) -> PhysicallyBasedMaterial {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: .black)
        m.emissiveColor = .init(color: color)
        m.emissiveIntensity = intensity
        return m
    }

    private static func mesh(_ name: String, _ positions: [SIMD3<Float>], _ indices: [UInt32]) -> MeshResource {
        var d = MeshDescriptor(name: name)
        d.positions = MeshBuffer(positions)
        d.primitives = .triangles(indices)
        return try! MeshResource.generate(from: [d])
    }
}

extension PhysicallyBasedMaterial {
    static func make(_ color: UIColor, roughness: Float) -> PhysicallyBasedMaterial {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: color)
        m.roughness = .init(floatLiteral: roughness)
        return m
    }
}

extension UIColor {
    static let holoCyan = UIColor(red: 0.3, green: 0.9, blue: 1, alpha: 1)
    static let holoGreen = UIColor(red: 0.3, green: 1, blue: 0.55, alpha: 1)
    static let holoAmber = UIColor(red: 1, green: 0.7, blue: 0.2, alpha: 1)
    static let holoRed = UIColor(red: 1, green: 0.3, blue: 0.25, alpha: 1)
    static let holoViolet = UIColor(red: 0.75, green: 0.45, blue: 1, alpha: 1)
}

/// Small deterministic RNG so the scene is the same every launch.
struct SeededRandom {
    private var state: UInt64
    init(seed: UInt64) { state = seed &* 6364136223846793005 &+ 1442695040888963407 }
    mutating func next() -> Float {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return Float(state >> 40) / Float(1 << 24)
    }
    mutating func unitVector() -> SIMD3<Float> {
        simd_normalize(SIMD3<Float>(next() * 2 - 1, next() * 2 - 1, next() * 2 - 1) + 1e-4)
    }
}
