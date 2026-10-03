import Foundation

/// One time base for every sensor: seconds on the mach (uptime) clock, the same domain
/// as `CACurrentMediaTime()`, `ProcessInfo.systemUptime`, `CMLogItem.timestamp`, and
/// (on iOS) the host clock used by `CMSampleBuffer` presentation timestamps.
public enum Clock {
    private static let timebase: mach_timebase_info_data_t = {
        var info = mach_timebase_info_data_t()
        mach_timebase_info(&info)
        return info
    }()

    /// Current time in seconds on the mach clock (excludes sleep, like CACurrentMediaTime).
    @inline(__always)
    public static func now() -> Double {
        let ticks = mach_absolute_time()
        return Double(ticks) * Double(timebase.numer) / Double(timebase.denom) * 1e-9
    }

    /// Convert raw mach ticks (e.g. from `CMClockGetHostTimeClock`-based timestamps
    /// expressed as ticks) to seconds.
    public static func seconds(fromMachTicks ticks: UInt64) -> Double {
        Double(ticks) * Double(timebase.numer) / Double(timebase.denom) * 1e-9
    }

    /// `CMLogItem.timestamp` (CMDeviceMotion etc.) is already seconds since boot on the
    /// same clock. Kept as a function so call sites document the conversion.
    @inline(__always)
    public static func fromMotionTimestamp(_ ts: TimeInterval) -> Double { ts }

    /// `CMSampleBuffer` PTS from an AVCaptureSession is on the session's `synchronizationClock`,
    /// which is the host-time clock by default; `CMTimeGetSeconds(pts)` is then directly
    /// comparable with `now()`. Pass the already-converted seconds here.
    @inline(__always)
    public static func fromHostClockSeconds(_ s: Double) -> Double { s }

    /// Convert a wall-clock `Date` (e.g. `CLLocation.timestamp`) into the mach domain using
    /// the current offset between wall clock and uptime. Accurate to a few ms (clock steps
    /// by NTP can shift it; good enough for ~1 Hz GNSS).
    public static func fromDate(_ date: Date) -> Double {
        let nowWall = Date().timeIntervalSince1970
        let nowMach = now()
        return nowMach - (nowWall - date.timeIntervalSince1970)
    }

    /// Wall-clock string for run ids: `yyyy-MM-dd_HH-mm-ss`.
    public static func runIdTimestamp(_ date: Date = Date()) -> String {
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        return f.string(from: date)
    }
}
