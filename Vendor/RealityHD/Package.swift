// swift-tools-version: 6.2
// Vendored subset of RealityHD (https://github.com/hunterh37/RealityHD, MIT) — library targets only.
import PackageDescription

let package = Package(
    name: "RealityHD",
    platforms: [.visionOS(.v26), .macOS(.v26), .iOS(.v26)],
    products: [
        .library(name: "RealityHD", targets: ["RealCore", "RealMaterials", "RealKit", "RealLibrary"]),
    ],
    targets: [
        .target(name: "RealCore"),
        .target(name: "RealMaterials", dependencies: ["RealCore"]),
        .target(name: "RealKit", dependencies: ["RealCore", "RealMaterials"]),
        .target(name: "RealLibrary", dependencies: ["RealCore", "RealMaterials", "RealKit"]),
    ],
    swiftLanguageModes: [.v5]
)
