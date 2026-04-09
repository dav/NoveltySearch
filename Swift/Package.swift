// swift-tools-version: 5.10
import PackageDescription

let package = Package(
    name: "NoveltySearch",
    platforms: [.macOS(.v14)],
    targets: [
        .executableTarget(name: "NoveltySearch", path: "Sources"),
    ]
)
