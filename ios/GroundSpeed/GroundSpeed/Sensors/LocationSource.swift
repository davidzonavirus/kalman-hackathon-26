import CoreLocation
import Foundation
import PhoneRuntime

/// GNSS speed for the slow correction. Samples with speed < 0 (invalid) are dropped.
/// Create and start on the main thread (CLLocationManager delivers on the creating run loop).
final class LocationSource: NSObject, CLLocationManagerDelegate {
    typealias Sink = (_ t: Double, _ speed: Double, _ speedAcc: Double, _ courseDeg: Double) -> Void

    private let manager = CLLocationManager()
    var onSample: Sink?
    /// Latest horizontal accuracy (m) and speed accuracy (m/s), -1 = unknown. Main thread.
    private(set) var horizontalAccuracy: Double = -1
    private(set) var speedAccuracy: Double = -1
    private(set) var authorization: CLAuthorizationStatus = .notDetermined

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        manager.distanceFilter = kCLDistanceFilterNone
        manager.activityType = .otherNavigation
        manager.pausesLocationUpdatesAutomatically = false
        authorization = manager.authorizationStatus
    }

    func start() {
        switch manager.authorizationStatus {
        case .notDetermined:
            manager.requestWhenInUseAuthorization()
        case .authorizedWhenInUse, .authorizedAlways:
            manager.startUpdatingLocation()
        default:
            break
        }
    }

    func stop() {
        manager.stopUpdatingLocation()
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        authorization = manager.authorizationStatus
        if authorization == .authorizedWhenInUse || authorization == .authorizedAlways {
            manager.startUpdatingLocation()
        }
    }

    private var lastFixDate = Date.distantPast

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        for loc in locations {
            horizontalAccuracy = loc.horizontalAccuracy
            speedAccuracy = loc.speedAccuracy
            // Core Location replays cached fixes (on start and when the signal drops) and
            // repeats the last fix with a new delivery; neither is a new speed measurement.
            guard loc.timestamp > lastFixDate, -loc.timestamp.timeIntervalSinceNow < 2 else { continue }
            lastFixDate = loc.timestamp
            guard loc.speed >= 0, loc.speedAccuracy >= 0 else { continue }
            let course = loc.course >= 0 ? loc.course : -1
            onSample?(Clock.fromDate(loc.timestamp), loc.speed, loc.speedAccuracy, course)
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        // Indoors this is common (kCLErrorLocationUnknown); keep trying.
    }
}
