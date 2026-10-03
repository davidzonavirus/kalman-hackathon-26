// kfreplay — deterministic replay of a recorded run through any registered filter.
//
//   kfreplay <run_dir> [--filter NAME | --all] [--out FILE] [--config cfg.json]
//            [--zupt-detector auto|on|off] [--compare est.csv]
//   kfreplay --synth <out_dir> [--seed N] [--no-dropout]
//
// Replay: reads imu.csv / flow.csv / gnss.csv / events.csv, merges by timestamp (equal t:
// IMU predict, flow, GNSS, ZUPT) and writes est.csv with the frozen header (default
// <run_dir>/est_replay_<filter>.csv so the phone's est.csv is never overwritten).
//
// ZUPT sources: events.csv rows with event "zupt" (the app logs every ZUPT it applies) plus,
// optionally, KalmanCore.ZuptDetector run on the logged IMU/flow.
//   --zupt-detector auto (default): ON if events.csv has no "zupt" rows (e.g. synthetic runs
//                   or logs from builds that don't record ZUPTs), OFF otherwise — so replaying
//                   a phone log reproduces its est.csv exactly, and raw logs still get ZUPTs.
//   --zupt-detector on|off: force.
// Config: --config file (FilterConfig JSON, missing keys = defaults); else meta.json "q"/"r"
// objects (same snake_case keys); else defaults. Filter: --filter, else meta.json
// filter_name, else FilterRegistry.defaultName.
// --compare: max |Δv| against another est.csv (default: <run_dir>/est.csv if present), rows
// matched by t (exact text match of %.6f).

import Foundation
import KalmanCore
import SpeedProtocol

func die(_ msg: String) -> Never {
    FileHandle.standardError.write(Data(("kfreplay: " + msg + "\n").utf8))
    exit(2)
}

let usage = """
usage: kfreplay <run_dir> [--filter NAME | --all] [--out FILE] [--config cfg.json]
                [--zupt-detector auto|on|off] [--compare est.csv]
       kfreplay --synth <out_dir> [--seed N] [--no-dropout]
filters: \(FilterRegistry.names.joined(separator: ", "))
"""

// MARK: - CSV IO

/// Parses a CSV with a header into column-name → index plus rows of cells.
func readCSV(_ url: URL) -> (cols: [String: Int], rows: [[String]])? {
    guard let text = try? String(contentsOf: url, encoding: .utf8) else { return nil }
    var lines = text.split(whereSeparator: { $0 == "\n" || $0 == "\r\n" })
    guard !lines.isEmpty else { return nil }
    let header = CSVRow.split(lines.removeFirst()).map { $0.trimmingCharacters(in: .whitespaces) }
    var cols: [String: Int] = [:]
    for (i, h) in header.enumerated() { cols[h] = i }
    let rows = lines.filter { !$0.trimmingCharacters(in: .whitespaces).isEmpty }.map { CSVRow.split($0) }
    return (cols, rows)
}

func numeric(_ csv: (cols: [String: Int], rows: [[String]]), _ names: [String], file: String) -> [[Double]] {
    let idx = names.map { n -> Int in
        guard let i = csv.cols[n] else { die("\(file): missing column '\(n)'") }
        return i
    }
    var out: [[Double]] = []
    out.reserveCapacity(csv.rows.count)
    for (ln, r) in csv.rows.enumerated() {
        var v: [Double] = []
        for i in idx {
            guard i < r.count, let d = Double(r[i].trimmingCharacters(in: .whitespaces)) else {
                FileHandle.standardError.write(Data("kfreplay: \(file): skipping bad row \(ln + 2)\n".utf8))
                v = []; break
            }
            v.append(d)
        }
        if !v.isEmpty { out.append(v) }
    }
    return out
}

struct RunData {
    var imu: [ImuSample] = []
    var flow: [FlowSample] = []
    var gnss: [GnssSample] = []
    var events: [RunEvent] = []
    var meta: RunMeta?
    var metaRaw: [String: Any]?
}

func loadRun(_ dir: URL) -> RunData {
    var d = RunData()
    guard let imu = readCSV(dir.appendingPathComponent(CSVSchema.imuFile)) else { die("cannot read \(CSVSchema.imuFile) in \(dir.path)") }
    d.imu = numeric(imu, ["t", "ax", "ay", "az", "gx", "gy", "gz"], file: CSVSchema.imuFile)
        .map { ImuSample(t: $0[0], ax: $0[1], ay: $0[2], az: $0[3], gx: $0[4], gy: $0[5], gz: $0[6]) }
    if let flow = readCSV(dir.appendingPathComponent(CSVSchema.flowFile)) {
        d.flow = numeric(flow, ["t", "vx_cam", "vy_cam", "quality", "h"], file: CSVSchema.flowFile)
            .map { FlowSample(t: $0[0], vx: $0[1], vy: $0[2], quality: $0[3], h: $0[4]) }
    }
    if let g = readCSV(dir.appendingPathComponent(CSVSchema.gnssFile)) {
        d.gnss = numeric(g, ["t", "speed", "speed_acc", "course"], file: CSVSchema.gnssFile)
            .map { GnssSample(t: $0[0], speed: $0[1], speedAcc: $0[2], course: $0[3]) }
    }
    if let e = readCSV(dir.appendingPathComponent(CSVSchema.eventsFile)),
       let it = e.cols["t"], let ie = e.cols["event"] {
        let iv = e.cols["value"]
        for r in e.rows {
            guard it < r.count, ie < r.count, let t = Double(r[it]) else { continue }
            let v = iv.flatMap { $0 < r.count ? r[$0] : nil } ?? ""
            d.events.append(RunEvent(t: t, event: r[ie].trimmingCharacters(in: .whitespaces), value: v))
        }
    }
    if let data = try? Data(contentsOf: dir.appendingPathComponent(CSVSchema.metaFile)) {
        d.meta = try? JSONDecoder().decode(RunMeta.self, from: data)
        d.metaRaw = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any]
    }
    return d
}

func statusBits(_ r: EstRow) -> UInt16 {
    var s: StatusFlags = [.imuOK]
    if r.initialized { s.insert(.filterInit) }
    if r.flowOK { s.insert(.flowOK) }
    if r.flowGated { s.insert(.flowGated) }
    if r.gnssOK { s.insert(.gnssOK) }
    if r.gnssGated { s.insert(.gnssGated) }
    if r.zupt { s.insert(.zupt) }
    return s.rawValue
}

func estLine(_ r: EstRow) -> String {
    CSVRow.join([CSVRow.time(r.t), CSVRow.real(r.vx), CSVRow.real(r.vy), CSVRow.real(r.sigmaVx),
                 CSVRow.real(r.sigmaVy), CSVRow.real(r.distance), String(statusBits(r)),
                 CSVRow.real(r.flowQuality), CSVRow.real(r.h)])
}

func write(_ text: String, to url: URL) {
    do { try text.write(to: url, atomically: true, encoding: .utf8) }
    catch { die("cannot write \(url.path): \(error)") }
}

// MARK: - Synthetic run

func writeSynthetic(to dir: URL, seed: UInt64, dropout: Bool) {
    try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    var cfg = SyntheticRun.Config()
    cfg.seed = seed
    if !dropout { cfg.dropout = nil }
    let run = SyntheticRun(config: cfg)

    var s = CSVSchema.imuHeader + "\n"
    for x in run.imu { s += CSVRow.format(t: x.t, [x.ax, x.ay, x.az, x.gx, x.gy, x.gz]) + "\n" }
    write(s, to: dir.appendingPathComponent(CSVSchema.imuFile))
    s = CSVSchema.flowHeader + "\n"
    for x in run.flow { s += CSVRow.format(t: x.t, [x.vx, x.vy, x.quality, x.h]) + "\n" }
    write(s, to: dir.appendingPathComponent(CSVSchema.flowFile))
    write(CSVSchema.gnssHeader + "\n", to: dir.appendingPathComponent(CSVSchema.gnssFile))
    s = CSVSchema.depthHeader + "\n"
    for x in run.depth { s += CSVRow.format(t: x.t, [x.h]) + "\n" }
    write(s, to: dir.appendingPathComponent(CSVSchema.depthFile))
    s = CSVSchema.eventsHeader + "\n"
    for e in run.events { s += CSVRow.join([CSVRow.time(e.t), CSVRow.text(e.event), CSVRow.text(e.value)]) + "\n" }
    write(s, to: dir.appendingPathComponent(CSVSchema.eventsFile))
    // Ground truth (not part of the phone format; for plotting / scoring).
    s = "t,v_true,distance_true\n"
    for x in run.truth { s += CSVRow.format(t: x.t, [x.v, x.d]) + "\n" }
    write(s, to: dir.appendingPathComponent("truth.csv"))

    let cfgF = FilterConfig.default
    let meta = RunMeta(runId: dir.lastPathComponent, label: "synthetic_10m", appVersion: "synthetic",
                       device: "kfreplay --synth seed=\(seed)", filterName: FilterRegistry.defaultName,
                       mountHeightM: cfg.height, torchLevel: 0, scaleFactor: 1, focalPx: 1450,
                       mountConfig: .object(["swap_xy": .bool(false), "flip_x": .bool(false), "flip_y": .bool(true)]),
                       startT: run.startT, endT: run.endT, batteryStart: 100, batteryEnd: 99,
                       q: cfgF.qDict, r: cfgF.rDict)
    do { try meta.encodedJSON().write(to: dir.appendingPathComponent(CSVSchema.metaFile)) }
    catch { die("cannot write meta.json: \(error)") }

    // Reload from the CSV text (what a real consumer sees) and produce the app-style est.csv.
    let data = loadRun(dir)
    let f = FilterRegistry.make(name: FilterRegistry.defaultName)!
    var est = CSVSchema.estHeader + "\n"
    let sum = Replay.run(filter: f, imu: data.imu, flow: data.flow, gnss: data.gnss, events: data.events,
                         zuptDetector: ZuptDetector(), onRow: { est += estLine($0) + "\n" })
    write(est, to: dir.appendingPathComponent(CSVSchema.estFile))
    print("synthetic run → \(dir.path)")
    print(String(format: "  %.2f s, imu %d, flow %d, truth distance %.4f m, %@ est distance %.4f m",
                 run.totalDuration, run.imu.count, run.flow.count, run.truth.last?.d ?? 0,
                 FilterRegistry.defaultName, sum.finalDistance))
    if let w = run.dropoutWindow {
        print(String(format: "  lens-covered dropout t = %.3f … %.3f", w.lowerBound, w.upperBound))
    }
}

// MARK: - Main

var args = Array(CommandLine.arguments.dropFirst())
func take(_ flag: String) -> String? {
    guard let i = args.firstIndex(of: flag) else { return nil }
    guard i + 1 < args.count else { die("\(flag) needs a value") }
    let v = args[i + 1]
    args.removeSubrange(i...(i + 1))
    return v
}
func flag(_ f: String) -> Bool {
    guard let i = args.firstIndex(of: f) else { return false }
    args.remove(at: i); return true
}

if args.isEmpty || flag("-h") || flag("--help") { print(usage); exit(args.isEmpty ? 2 : 0) }

if let out = take("--synth") {
    let seed = take("--seed").flatMap { UInt64($0) } ?? 42
    writeSynthetic(to: URL(fileURLWithPath: out), seed: seed, dropout: !flag("--no-dropout"))
    exit(0)
}

let filterArg = take("--filter")
let outArg = take("--out")
let configArg = take("--config")
let zuptArg = take("--zupt-detector") ?? "auto"
let compareArg = take("--compare")
let runAll = flag("--all")
if flag("--no-zupt-detector") { die("use --zupt-detector off") }
guard args.count == 1 else { die("expected exactly one run_dir\n" + usage) }
let runDir = URL(fileURLWithPath: args[0])
let data = loadRun(runDir)

// Config
var config = FilterConfig.default
if let path = configArg {
    guard let d = try? Data(contentsOf: URL(fileURLWithPath: path)) else { die("cannot read \(path)") }
    do { config = try JSONDecoder().decode(FilterConfig.self, from: d) } catch { die("bad config: \(error)") }
} else if let m = data.meta {
    let merged = m.q.merging(m.r) { a, _ in a }
    if let d = try? JSONEncoder().encode(merged), let c = try? JSONDecoder().decode(FilterConfig.self, from: d) {
        config = c
    }
}

let hasZuptEvents = data.events.contains { $0.event == CSVSchema.Event.zupt }
let useDetector: Bool
switch zuptArg {
case "on": useDetector = true
case "off": useDetector = false
case "auto": useDetector = !hasZuptEvents
default: die("--zupt-detector must be auto|on|off")
}

let names: [String]
if runAll { names = FilterRegistry.names }
else {
    let n = filterArg ?? data.meta?.filterName ?? FilterRegistry.defaultName
    guard FilterRegistry.all[n] != nil else { die("unknown filter '\(n)'; have: \(FilterRegistry.names.joined(separator: ", "))") }
    names = [n]
}

// Reference est.csv for comparison (keyed by formatted t).
var reference: [String: (vx: Double, vy: Double)] = [:]
let compareURL = compareArg.map { URL(fileURLWithPath: $0) } ?? runDir.appendingPathComponent(CSVSchema.estFile)
if let ref = readCSV(compareURL), let it = ref.cols["t"], let ivx = ref.cols["v_x"], let ivy = ref.cols["v_y"] {
    for r in ref.rows where r.count > max(it, ivx, ivy) {
        if let a = Double(r[ivx]), let b = Double(r[ivy]) { reference[r[it]] = (a, b) }
    }
}

print("run: \(runDir.path)")
print("  imu \(data.imu.count), flow \(data.flow.count), gnss \(data.gnss.count), events \(data.events.count)"
      + " (zupt events: \(data.events.filter { $0.event == "zupt" }.count)), zupt detector: \(useDetector ? "on" : "off")")
func pad(_ s: String, _ w: Int, left: Bool = false) -> String {
    let p = String(repeating: " ", count: max(0, w - s.count))
    return left ? s + p : p + s
}
print("  " + pad("filter", 16, left: true) + pad("distance m", 12) + pad("max σv", 9) + pad("flow acc/gated/skip", 21)
      + pad("zupts", 7) + "  max|Δv| vs ref est")

for name in names {
    let f = FilterRegistry.make(name: name, config: config)!
    var out = CSVSchema.estHeader + "\n"
    var maxDv = 0.0, matched = 0
    let s = Replay.run(filter: f, imu: data.imu, flow: data.flow, gnss: data.gnss, events: data.events,
                       zuptDetector: useDetector ? ZuptDetector() : nil) { row in
        let line = estLine(row)
        out += line + "\n"
        if let ref = reference[CSVRow.time(row.t)] {
            matched += 1
            maxDv = max(maxDv, abs(ref.vx - Double(CSVRow.real(row.vx))!), abs(ref.vy - Double(CSVRow.real(row.vy))!))
        }
    }
    let outURL: URL
    if let o = outArg, !runAll { outURL = URL(fileURLWithPath: o) }
    else { outURL = runDir.appendingPathComponent("est_replay_\(name).csv") }
    write(out, to: outURL)
    let cmp = matched > 0 ? String(format: "%.3g (%d rows)", maxDv, matched) : "n/a"
    print("  " + pad(name, 16, left: true) + pad(String(format: "%.4f", s.finalDistance), 12)
          + pad(String(format: "%.4f", s.maxSigmaV), 9)
          + pad("\(s.flowAccepted)/\(s.flowGated)/\(s.flowSkipped)", 21) + pad("\(s.zuptCount)", 7) + "  " + cmp)
    print("    → \(outURL.path)")
}
