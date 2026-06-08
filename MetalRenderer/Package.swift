// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "FTCSimRenderer",
    platforms: [.macOS(.v13)],
    targets: [
        .executableTarget(
            name: "FTCSimRenderer",
            path: "Sources/FTCSimRenderer",
            resources: [.process("Shaders.metal")]
        )
    ]
)
