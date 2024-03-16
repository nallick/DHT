// swift-tools-version:5.1

import PackageDescription

let package = Package(
    name: "DHT",
    platforms: [.macOS(.v10_12)],
    products: [
        .library(
            name: "DHT",
            targets: ["DHT"]),
    ],
    dependencies: [
        .package(url: "https://github.com/nallick/SwiftyGPIO.git", from: "1.5.0"),
    ],
    targets: [
        .target(
            name: "DHT",
            dependencies: ["SwiftyGPIO"]),
    ]
)
