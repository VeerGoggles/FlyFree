import RealityKit
import UIKit

/// Fast-changing numeric text (clocks, readouts). Every slot holds one pre-built entity per
/// supported character; updating the text only toggles which one is visible. Nothing is
/// generated or swapped at runtime, so values can change every frame and always render.
@MainActor
final class GlyphText {
    let entity = Entity()
    private var slots: [[Character: ModelEntity]] = []
    private var shown: [Character?] = []
    private let advance: Float

    private static let characters = Array("0123456789:.-k")
    private static var cache: [String: MeshResource] = [:]

    init(size: CGFloat, color: UIColor, maxCharacters: Int = 8) {
        let font = UIFont.monospacedDigitSystemFont(ofSize: size, weight: .semibold)
        let material = MeshFactory.glow(color, opacity: 0.95)
        advance = Float(size) * 0.62
        for _ in 0..<maxCharacters {
            let cell = Entity()
            entity.addChild(cell)
            var glyphs: [Character: ModelEntity] = [:]
            for c in Self.characters {
                let mesh = Self.glyph(c, font: font)
                let e = ModelEntity(mesh: mesh, materials: [material])
                e.position = [-mesh.bounds.center.x, -mesh.bounds.center.y, 0]
                e.isEnabled = false
                cell.addChild(e)
                glyphs[c] = e
            }
            slots.append(glyphs)
            shown.append(nil)
        }
    }

    func set(_ text: String) {
        let characters = Array(text.prefix(slots.count))
        let width = Float(characters.count) * advance
        for i in slots.indices {
            let wanted: Character? = i < characters.count ? characters[i] : nil
            if let cell = slots[i].values.first?.parent {
                cell.position = [-width / 2 + (Float(i) + 0.5) * advance, 0, 0]
            }
            guard wanted != shown[i] else { continue }
            if let old = shown[i] { slots[i][old]?.isEnabled = false }
            if let new = wanted { slots[i][new]?.isEnabled = true }
            shown[i] = wanted
        }
    }

    private static func glyph(_ character: Character, font: UIFont) -> MeshResource {
        let key = "\(font.pointSize)|\(character)"
        if let mesh = cache[key] { return mesh }
        let mesh = MeshResource.generateText(String(character), extrusionDepth: 0.0005, font: font)
        cache[key] = mesh
        return mesh
    }
}
