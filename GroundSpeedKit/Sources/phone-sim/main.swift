// phone-sim: a simulated phone built on the REAL PhoneRuntime (engine, recorder, UDP
// telemetry, TCP command server). Plays a synthetic 10.00 m push in real time, looping.
//
//   swift run phone-sim [--host 127.0.0.1] [--port 9000] [--cmd-port 9001] [--json]
//                       [--runs-dir /tmp/gsk-runs] [--once] [--record] [--loops N] [--quiet]
//
// --once    play one push, auto-recording it, print the result and exit.
// --record  auto start/stop a recorded run around every loop (otherwise use TCP start_run).
import Foundation
import KalmanCore
import PhoneRuntime
import SpeedProtocol

setvbuf(stdout, nil, _IOLBF, 0)

struct Args {
    var host = "127.0.0.1"
    var port: UInt16 = 9000
    var cmdPort: UInt16 = 9001
    var json = false
    var runsDir = URL(fileURLWithPath: NSTemporaryDirectory()).appendingPathComponent("gsk-sim-runs")
    var once = false
    var record = false
    var loops = Int.max
    var quiet = false
}

func usage() -> Never {
    print("""
    usage: phone-sim [--host H] [--port 9000] [--cmd-port 9001] [--json] [--runs-dir DIR]
                     [--once] [--record] [--loops N] [--quiet]
    """)
    exit(2)
}

var args = Args()
var it = CommandLine.arguments.dropFirst().makeIterator()
func value<T>(_ parse: (String) -> T?) -> T {
    guard let s = it.next(), let v = parse(s) else { usage() }
    return v
}
while let a = it.next() {
    switch a {
    case "--host": args.host = value { $0 }
    case "--port": args.port = value { UInt16($0) }
    case "--cmd-port": args.cmdPort = value { UInt16($0) }
    case "--json": args.json = true
    case "--runs-dir": args.runsDir = value { URL(fileURLWithPath: $0) }
    case "--once": args.once = true; args.loops = 1; args.record = true
    case "--record": args.record = true
    case "--loops": args.loops = value { Int($0) }
    case "--quiet": args.quiet = true
    case "-h", "--help": usage()
    default: print("unknown argument \(a)"); usage()
    }
}

var settings = RuntimeSettings()
settings.dashboardHost = args.host
settings.telemetryPort = args.port
settings.commandPort = args.cmdPort
settings.useJSON = args.json
settings.manualHeight = 0.30

let runtime = GroundSpeedRuntime(settings: settings, runsDirectory: args.runsDir, bonjourName: "phone-sim")
var hooks = GroundSpeedRuntime.Hooks()
hooks.battery = { 87 }
hooks.setTorch = { $0 }
hooks.focalPx = { 1450 }
hooks.deviceName = "phone-sim (\(ProcessInfo.processInfo.hostName))"
hooks.appVersion = "phone-sim 0.1"
runtime.hooks = hooks
runtime.setTorch(level: settings.torchLevel)

do {
    try runtime.start()
} catch {
    print("command server failed to start: \(error) (telemetry still running)")
}
_ = runtime.server.waitUntilReady(timeout: 2)
print("phone-sim: UDP → \(args.host):\(args.port) (\(args.json ? "JSON" : "binary")), TCP commands on :\(args.cmdPort) [\(runtime.server.state)], runs → \(args.runsDir.path)")

signal(SIGINT) { _ in
    // Best effort: close an active run so its files are complete.
    if runtime.isRecording { runtime.stopRun() }
    exit(0)
}

let synth = SimPush(config: SimPush.Config())
print(String(format: "synthetic push: %.1f s, true distance %.2f m, flow dropout %@",
             synth.duration, synth.config.distance,
             synth.config.dropout.map { String(format: "%.1f–%.1f s", $0.lowerBound, $0.upperBound) } ?? "none"))

var loop = 0
while loop < args.loops {
    loop += 1
    // Distance is never reset here: it is "since run start" (start_run / --record) or since the
    // last reset_distance / launch, exactly like the phone.
    var runId: String?
    if args.record {
        runId = try? runtime.startRun(label: "sim\(loop)")
        print("[loop \(loop)] recording \(runId ?? "?")")
    }
    let t0 = Clock.now() + 0.05
    var nextPrint = 0.0
    for s in synth.samples {
        let target = t0 + s.t
        let wait = target - Clock.now()
        if wait > 0.0005 { Thread.sleep(forTimeInterval: wait) }
        SimPush.feed(s, t0: t0, into: runtime.engine)
        if !args.quiet, s.t >= nextPrint {
            nextPrint += 1
            // Same source as the UDP telemetry: the frame the sender would transmit now.
            let f = runtime.makeFrame()
            let truth = SimPush.truth(synth.config, s.t)
            print(String(format: "[loop %d] t=%5.1f  v_x=%6.3f (true %5.3f) ±%5.3f  dist=%6.3f (push %6.3f)  psr=%5.1f  status=%@  sent=%llu",
                         loop, s.t, f.vx, truth.v, f.sigmaVx, f.distance, truth.x, f.flowQuality,
                         f.status.names.joined(separator: "|"), runtime.sender.framesSent))
            if let e = runtime.fpgaEstimate(maxAge: 1.0) {
                print(String(format: "           FPGA estimate back (%llu): v_x=%6.3f ±%5.3f  dist=%6.3f  [%@]",
                             runtime.fpgaEstimatesReceived, e.vx, e.sigmaVx, e.distance, e.backend))
            }
        }
    }
    runtime.engine.drain()
    let snap = runtime.engine.snapshot()
    let f = runtime.makeFrame()
    print(String(format: "[loop %d] done: telemetry distance %.3f m (each push is %.2f m true; distance resets only on start_run / reset_distance), zupts %d, flow gated %d",
                 loop, f.distance, synth.config.distance, snap.zuptCount, snap.flowGatedCount))
    if args.record, let id = runtime.stopRun() {
        print("[loop \(loop)] saved \(args.runsDir.appendingPathComponent(id).path)")
    }
}
// Let the last telemetry frames go out.
Thread.sleep(forTimeInterval: 0.2)
print("phone-sim: frames sent \(runtime.sender.framesSent), send errors \(runtime.sender.sendErrors)")
runtime.stop()
