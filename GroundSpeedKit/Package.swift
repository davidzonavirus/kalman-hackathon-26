// swift-tools-version:5.9
import PackageDescription

// Checks run as an executable rather than XCTest so they also work with Command Line Tools
// alone: `swift run gsk-checks` exits non-zero on any failure.
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
        // Telemetry frame, CRC, dashboard commands, run CSV schema
        .target(name: "SpeedProtocol"),
        // GroundSpeedFilter protocol, Kalman filter variants, registry, replay
        .target(name: "KalmanCore"),
        // Phase-correlation optical flow and height tracking (Accelerate/vDSP)
        .target(name: "OpticalFlow"),
        // Platform-independent phone runtime: fusion engine, recorder, UDP telemetry, TCP commands
        .target(name: "PhoneRuntime", dependencies: ["SpeedProtocol", "KalmanCore", "OpticalFlow"]),
        .executableTarget(name: "gsk-checks", dependencies: ["SpeedProtocol", "KalmanCore", "OpticalFlow", "PhoneRuntime"]),
        // Replays a recorded run through any filter; generates synthetic runs
        .executableTarget(name: "kfreplay", dependencies: ["SpeedProtocol", "KalmanCore"]),
        // Speed-range benchmark: PhaseCorrelator on a synthetic floor, shift sweep with motion blur
        .executableTarget(name: "flowbench", dependencies: ["OpticalFlow"]),
        // Simulated phone on the real PhoneRuntime: streams to the dashboard, answers commands
        .executableTarget(name: "phone-sim", dependencies: ["SpeedProtocol", "KalmanCore", "PhoneRuntime"]),
    ]
)
