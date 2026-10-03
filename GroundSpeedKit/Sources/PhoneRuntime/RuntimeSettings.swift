import Foundation
import KalmanCore
import OpticalFlow

/// Everything the user can change at runtime (persisted by the app as JSON in UserDefaults).
/// Missing keys decode to defaults so older saved settings keep loading.
public struct RuntimeSettings: Codable, Sendable, Equatable {
    /// Dashboard laptop IP. With iPhone Personal Hotspot the laptop is usually 172.20.10.x.
    /// Overridden automatically when a dashboard connects on the TCP command port.
    public var dashboardHost: String = "172.20.10.2"
    public var telemetryPort: UInt16 = 9000
    public var commandPort: UInt16 = 9001
    /// Send JSON debug datagrams instead of binary frames.
    public var useJSON: Bool = false
    /// Camera height above the floor (m) used when LiDAR is unavailable.
    public var manualHeight: Double = 0.30
    /// Use LiDAR depth for height when available.
    public var useLiDAR: Bool = true
    public var imuMapping: IMUMountMapping = IMUMountMapping()
    public var flowMapping: MountMapping = MountMapping()
    public var filterName: String = FilterRegistry.defaultName
    public var filterConfig: FilterConfig = .default
    /// Empirical flow scale correction (from a measured push), 1 = none.
    public var scaleFactor: Double = 1.0
    /// Extra PSR floor for FLOW_OK / ZUPT flow confirmation; effective threshold is
    /// max(this, filterConfig.psrMin). 0 = follow the filter's psr_min.
    public var flowQualityThreshold: Double = 0
    /// Torch level 0…1 (0 = off).
    public var torchLevel: Double = 0.5
    public var zuptEnabled: Bool = true

    public init() {}

    enum CodingKeys: String, CodingKey {
        case dashboardHost, telemetryPort, commandPort, useJSON, manualHeight, useLiDAR
        case imuMapping, flowMapping, filterName, filterConfig, scaleFactor
        case flowQualityThreshold, torchLevel, zuptEnabled
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let d = RuntimeSettings()
        dashboardHost = (try? c.decodeIfPresent(String.self, forKey: .dashboardHost)) ?? d.dashboardHost
        telemetryPort = (try? c.decodeIfPresent(UInt16.self, forKey: .telemetryPort)) ?? d.telemetryPort
        commandPort = (try? c.decodeIfPresent(UInt16.self, forKey: .commandPort)) ?? d.commandPort
        useJSON = (try? c.decodeIfPresent(Bool.self, forKey: .useJSON)) ?? d.useJSON
        manualHeight = (try? c.decodeIfPresent(Double.self, forKey: .manualHeight)) ?? d.manualHeight
        useLiDAR = (try? c.decodeIfPresent(Bool.self, forKey: .useLiDAR)) ?? d.useLiDAR
        imuMapping = (try? c.decodeIfPresent(IMUMountMapping.self, forKey: .imuMapping)) ?? d.imuMapping
        flowMapping = (try? c.decodeIfPresent(MountMapping.self, forKey: .flowMapping)) ?? d.flowMapping
        filterName = (try? c.decodeIfPresent(String.self, forKey: .filterName)) ?? d.filterName
        filterConfig = (try? c.decodeIfPresent(FilterConfig.self, forKey: .filterConfig)) ?? d.filterConfig
        scaleFactor = (try? c.decodeIfPresent(Double.self, forKey: .scaleFactor)) ?? d.scaleFactor
        flowQualityThreshold = (try? c.decodeIfPresent(Double.self, forKey: .flowQualityThreshold)) ?? d.flowQualityThreshold
        torchLevel = (try? c.decodeIfPresent(Double.self, forKey: .torchLevel)) ?? d.torchLevel
        zuptEnabled = (try? c.decodeIfPresent(Bool.self, forKey: .zuptEnabled)) ?? d.zuptEnabled
    }

    /// Engine-side subset.
    public var engineSettings: SensorFusionEngine.Settings {
        var s = SensorFusionEngine.Settings()
        s.flowQualityThreshold = flowQualityThreshold
        s.manualHeight = manualHeight
        s.zuptEnabled = zuptEnabled
        return s
    }

    public func encoded() -> Data {
        (try? JSONEncoder().encode(self)) ?? Data()
    }

    public static func decode(_ data: Data?) -> RuntimeSettings {
        guard let data, let s = try? JSONDecoder().decode(RuntimeSettings.self, from: data) else {
            return RuntimeSettings()
        }
        return s
    }
}
