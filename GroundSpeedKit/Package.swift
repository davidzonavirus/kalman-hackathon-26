// swift-tools-version:5.9
import PackageDescription

// NOTE: no XCTest/Testing module with Command Line Tools only, so checks live in the
// `gsk-checks` executable: `swift run gsk-checks` (exit code != 0 on failure).
let package = Package(
    name: "GroundSpeedKit",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "SpeedProtocol", targets: ["SpeedProtocol"]),
        .library(name: "KalmanCore", targets: ["KalmanCore"]),
        .library(name: "OpticalFlow", targets: ["OpticalFlow"]),
        .library(name: "PhoneRuntime", targets: ["PhoneRuntime"]),
    ],
    targets: [
        // Agent A: wire format, CRC, commands, CSV schema
        .target(name: "SpeedProtocol"),
        // Agent A: GroundSpeedFilter protocol, ReferenceKF4, registry (Joseph plugs in here)
        .target(name: "KalmanCore"),
        // Agent A: phase-correlation optical flow (Accelerate/vDSP)
        .target(name: "OpticalFlow"),
        // Agent B: platform-agnostic phone runtime (fusion pipeline, recorder, UDP, TCP cmds)
        .target(name: "PhoneRuntime", dependencies: ["SpeedProtocol", "KalmanCore", "OpticalFlow"]),
        .executableTarget(name: "gsk-checks", dependencies: ["SpeedProtocol", "KalmanCore", "OpticalFlow", "PhoneRuntime"]),
        .executableTarget(name: "kfreplay", dependencies: ["SpeedProtocol", "KalmanCore"]),
        // Simulated phone using the REAL PhoneRuntime: streams to the dashboard, answers TCP commands
        .executableTarget(name: "phone-sim", dependencies: ["SpeedProtocol", "KalmanCore", "PhoneRuntime"]),
    ]
)
