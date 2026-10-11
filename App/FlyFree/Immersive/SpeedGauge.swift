import FlyFreeCore
import RealityKit
import UIKit

/// A round speedometer: dial face, rim, scale ticks with labels, a needle and a digital
/// readout. The scale comes from the craft (`GaugeScale`): linear airspeed for the jet,
/// logarithmic for the spacecraft. Faces +Z; mount it angled toward the pilot.
@MainActor
final class SpeedGauge {
    let entity = Entity()
    private let needle = Entity()
    private let readout = ModelEntity()
    private let scale: GaugeScale
    private let radius: Float = 0.075
    private var shownValue: Int?
    private var smoothedFraction: Float = 0

    /// 270° sweep from lower-left (0) clockwise to lower-right (max).
    private static let sweep: Float = 1.5 * .pi
    private static func angle(_ fraction: Double) -> Float { 0.75 * .pi - Float(fraction) * sweep }

    init(scale: GaugeScale) {
        self.scale = scale
        let face = ModelEntity(mesh: .generateCylinder(height: 0.004, radius: radius + 0.008),
                               materials: [MeshFactory.metal(0.06, roughness: 0.35)])
        face.orientation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
        face.position = [0, 0, -0.003]
        entity.addChild(face)
        let rim = ModelEntity(mesh: MeshFactory.annulus(inner: radius + 0.004, outer: radius + 0.009),
                              materials: [MeshFactory.glow(.holoCyan, opacity: 0.85)])
        rim.orientation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
        entity.addChild(rim)

        let tickMaterial = MeshFactory.glow(.white, opacity: 0.95)
        let redline = MeshFactory.glow(.holoRed, opacity: 0.95)
        for tick in scale.ticks {
            let a = Self.angle(tick.fraction)
            let length: Float = tick.major ? 0.014 : 0.007
            let bar = ModelEntity(mesh: .generateBox(size: [tick.major ? 0.0022 : 0.0012, length, 0.001]),
                                  materials: [tick.fraction > 0.92 ? redline : tickMaterial])
            let r = radius - length / 2
            bar.position = SIMD3(-sin(a) * r, cos(a) * r, 0.001)
            bar.orientation = simd_quatf(angle: a, axis: [0, 0, 1])
            entity.addChild(bar)
            if let label = tick.label {
                let text = Self.text(label, size: 0.0095, color: .white)
                let lr = radius - 0.026
                text.position += SIMD3(-sin(a) * lr, cos(a) * lr, 0.001)
                entity.addChild(text)
            }
        }

        let title = Self.text(scale.title, size: 0.008, color: .holoCyan)
        title.position += [0, 0.03, 0.001]
        entity.addChild(title)
        let unit = Self.text(scale.unit, size: 0.007, color: UIColor(white: 0.7, alpha: 1))
        unit.position += [0, -0.05, 0.001]
        entity.addChild(unit)
        readout.position = [0, -0.035, 0.001]
        entity.addChild(readout)

        // Needle: pivots about the centre (the needle points along +Y at angle 0).
        let blade = ModelEntity(mesh: .generateBox(size: [0.0028, radius * 0.92, 0.0015], cornerRadius: 0.0007),
                                materials: [MeshFactory.glow(.holoAmber)])
        blade.position = [0, radius * 0.46 - 0.008, 0.003]
        needle.addChild(blade)
        let hub = ModelEntity(mesh: .generateSphere(radius: 0.006), materials: [MeshFactory.metal(0.5, roughness: 0.2)])
        hub.position = [0, 0, 0.003]
        needle.addChild(hub)
        entity.addChild(needle)
        needle.orientation = simd_quatf(angle: Self.angle(0), axis: [0, 0, 1])
    }

    func update(speed: Double) {
        // Ease the needle a little so tracking jitter doesn't make it buzz.
        let target = Float(scale.fraction(for: speed))
        smoothedFraction += (target - smoothedFraction) * 0.25
        needle.orientation = simd_quatf(angle: Self.angle(Double(smoothedFraction)), axis: [0, 0, 1])

        let rounded = speed >= 1000 ? Int(speed / 10) * 10 : Int(speed.rounded())
        guard rounded != shownValue else { return }
        shownValue = rounded
        let text = speed >= 10_000 ? String(format: "%.1fk", speed / 1000) : "\(rounded)"
        let mesh = MeshResource.generateText(text, extrusionDepth: 0.0005,
                                             font: .monospacedDigitSystemFont(ofSize: 0.014, weight: .semibold))
        readout.model = ModelComponent(mesh: mesh, materials: [MeshFactory.glow(.holoGreen)])
        readout.position = [-mesh.bounds.center.x, -0.035 - mesh.bounds.center.y, 0.001]
    }

    private static func text(_ string: String, size: CGFloat, color: UIColor) -> ModelEntity {
        let mesh = MeshResource.generateText(string, extrusionDepth: 0.0005, font: .systemFont(ofSize: size, weight: .semibold))
        let e = ModelEntity(mesh: mesh, materials: [MeshFactory.glow(color)])
        e.position = [-mesh.bounds.center.x, -mesh.bounds.center.y, 0]
        return e
    }
}
