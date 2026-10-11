// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "FlyFreeCore",
    platforms: [.visionOS(.v2), .macOS(.v15), .iOS(.v18)],
    products: [
        .library(name: "FlyFreeCore", targets: ["FlyFreeCore"]),
    ],
    targets: [
        .target(name: "FlyFreeCore"),
        .testTarget(name: "FlyFreeCoreTests", dependencies: ["FlyFreeCore"]),
    ]
)
