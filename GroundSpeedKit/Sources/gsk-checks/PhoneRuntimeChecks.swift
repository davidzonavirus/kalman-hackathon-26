import Foundation
import KalmanCore
import PhoneRuntime
import SpeedProtocol

// Agent B: PhoneRuntime checks (called from main.swift).
func phoneRuntimeChecks(_ r: inout CheckRunner) {
    let tmp = FileManager.default.temporaryDirectory
        .appendingPathComponent("gsk-checks-\(ProcessInfo.processInfo.processIdentifier)-\(Int(Date().timeIntervalSince1970))")
    defer { try? FileManager.default.removeItem(at: tmp) }

    recorderChecks(&r, dir: tmp.appendingPathComponent("rec"))
    axisMappingChecks(&r)
    engineSyntheticChecks(&r, dir: tmp.appendingPathComponent("engine"))
    distanceContinuityChecks(&r, dir: tmp.appendingPathComponent("continuity"))
    startMidMotionChecks(&r, dir: tmp.appendingPathComponent("midmotion"))
    senderRoundtripChecks(&r)
    commandServerChecks(&r, dir: tmp.appendingPathComponent("cmd"))
}

// MARK: - Recorder

private func recorderChecks(_ r: inout CheckRunner, dir: URL) {
    let rec = RunRecorder(runsDirectory: dir)
    var runURL: URL?
    do {
        let id = try rec.startRun(label: "steady carpet/03")
        r.check("recorder: run id has sanitized label", id.hasSuffix("_steady_carpet_03"), id)
        r.check("recorder: run id timestamp format",
                id.range(of: #"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_"#, options: .regularExpression) != nil, id)
        runURL = rec.currentRunURL
    } catch {
        r.check("recorder: startRun", false, "\(error)")
    }
    rec.recordIMU(t: 100.000001, ax: 0.1, ay: -0.2, az: 0.3, gx: 0.01, gy: 0.02, gz: 0.03)
    rec.recordFlow(t: 100.01, vx: 1.25, vy: -0.5, quality: 22.5, h: 0.3)
    rec.recordGNSS(t: 100.5, speed: 1.1, speedAcc: 0.4, courseDeg: 90)
    rec.recordDepth(t: 100.02, h: 0.301)
    rec.recordEstimate(t: 100.000001, vx: 1, vy: 0, sigmaVx: 0.1, sigmaVy: 0.1, distance: 2.5,
                       status: 0x0203, flowQuality: 22.5, h: 0.3)
    rec.recordEvent(t: 100.1, event: "mark", value: "lens, covered")
    let meta = RunMeta(runId: rec.currentRunId ?? "", filterName: "ReferenceKF4", mountHeightM: 0.3,
                       focalPx: 1400, startT: 100)
    let url = rec.stopRun(meta: meta)
    r.check("recorder: stopRun returns folder", url != nil && url == runURL)
    r.check("recorder: not recording after stop", !rec.isRecording)
    guard let url else { return }

    let expected: [(String, String)] = [
        (CSVSchema.imuFile, "t,ax,ay,az,gx,gy,gz"),
        (CSVSchema.flowFile, "t,vx_cam,vy_cam,quality,h"),
        (CSVSchema.gnssFile, "t,speed,speed_acc,course"),
        (CSVSchema.depthFile, "t,h"),
        (CSVSchema.estFile, "t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h"),
        (CSVSchema.eventsFile, "t,event,value"),
    ]
    for (file, header) in expected {
        let text = (try? String(contentsOf: url.appendingPathComponent(file), encoding: .utf8)) ?? ""
        let lines = text.split(separator: "\n", omittingEmptySubsequences: false)
        r.check("recorder: \(file) header exact", lines.first.map(String.init) == header, String(lines.first ?? "<missing>"))
        r.check("recorder: \(file) has 1 data row", lines.count == 3 && lines[2].isEmpty, "\(lines.count - 2) rows")
    }
    let imuRow = (try? String(contentsOf: url.appendingPathComponent(CSVSchema.imuFile), encoding: .utf8))?
        .split(separator: "\n").dropFirst().first.map(String.init) ?? ""
    r.check("recorder: imu row formatting (t %.6f, %.9g)", imuRow == "100.000001,0.1,-0.2,0.3,0.01,0.02,0.03", imuRow)
    let estRow = (try? String(contentsOf: url.appendingPathComponent(CSVSchema.estFile), encoding: .utf8))?
        .split(separator: "\n").dropFirst().first.map(String.init) ?? ""
    r.check("recorder: est status is integer", CSVRow.split(Substring(estRow))[6] == "515", estRow)
    let evRow = (try? String(contentsOf: url.appendingPathComponent(CSVSchema.eventsFile), encoding: .utf8))?
        .split(separator: "\n").dropFirst().first.map(String.init) ?? ""
    r.check("recorder: event text quoted/splittable", CSVRow.split(Substring(evRow)) == ["100.100000", "mark", "lens, covered"], evRow)
    let metaData = try? Data(contentsOf: url.appendingPathComponent(CSVSchema.metaFile))
    let decoded = metaData.flatMap { try? JSONDecoder().decode(RunMeta.self, from: $0) }
    r.check("recorder: meta.json decodes as RunMeta", decoded == meta)
    if let metaData, let obj = try? JSONSerialization.jsonObject(with: metaData) as? [String: Any] {
        let keys: Set<String> = ["run_id", "label", "app_version", "device", "filter_name", "mount_height_m",
                                 "torch_level", "scale_factor", "focal_px", "mount_config", "start_t", "end_t",
                                 "battery_start", "battery_end", "q", "r"]
        r.check("recorder: meta.json has exactly the PROTOCOL keys", Set(obj.keys) == keys, "\(obj.keys.sorted())")
    }
    r.check("recorder: listRuns finds the run", rec.listRuns().map(\.runId) == [url.lastPathComponent])
    do {
        let zip = try RunRecorder.zipRunFolder(url, destinationDirectory: dir)
        let head = (try? FileHandle(forReadingFrom: zip).readData(ofLength: 4)) ?? Data()
        r.check("recorder: zip export produces a PK archive", head.starts(with: [0x50, 0x4B]), zip.lastPathComponent)
    } catch {
        r.check("recorder: zip export", false, "\(error)")
    }
}

// MARK: - Mount mapping

private func axisMappingChecks(_ r: inout CheckRunner) {
    // Phone flat, screen up, top forward: pushing forward accelerates along device +y.
    let m = IMUMountMapping()
    let v = m.apply(x: 0, y: 1, z: 0.5)
    r.check("imu mapping: default device +y → vehicle +x", v.x == 1 && v.y == 0 && v.z == 0.5)
    let left = m.apply(x: -1, y: 0, z: 0)
    r.check("imu mapping: default device −x → vehicle +y (left)", left.x == 0 && left.y == 1)
    let flipped = IMUMountMapping(flipX: true).apply(x: 0, y: 1, z: 1)
    r.check("imu mapping: single flip is a reflection → z flips", flipped.x == -1 && flipped.z == -1)
}

// MARK: - Engine on the synthetic push (+ replay equivalence)

private func engineSyntheticChecks(_ r: inout CheckRunner, dir: URL) {
    for filterName in FilterRegistry.names {
        var settings = RuntimeSettings()
        settings.filterName = filterName
        let rt = GroundSpeedRuntime(settings: settings, runsDirectory: dir)
        let push = SimPush(config: SimPush.Config())
        let runId: String
        do { runId = try rt.startRun(label: filterName) } catch {
            r.check("engine[\(filterName)]: startRun", false, "\(error)"); continue
        }
        let t0 = Clock.now()
        for s in push.samples { SimPush.feed(s, t0: t0, into: rt.engine) }
        rt.engine.drain()
        let snap = rt.engine.snapshot()
        _ = rt.stopRun()
        r.near("engine[\(filterName)]: synthetic 10 m push distance", snap.distance, 10.0, tol: 0.15)
        r.near("engine[\(filterName)]: ends at rest", snap.vx, 0, tol: 0.03)
        r.check("engine[\(filterName)]: ZUPTs applied at rest", snap.zuptCount > 100, "\(snap.zuptCount)")
        r.check("engine[\(filterName)]: filter initialised", snap.status.contains(.filterInit))

        // Replay the logged CSVs through a fresh filter: must reproduce est.csv velocities.
        let url = dir.appendingPathComponent(runId)
        let imu = readCSV(url.appendingPathComponent(CSVSchema.imuFile)).map {
            ImuSample(t: $0[0], ax: $0[1], ay: $0[2], az: $0[3], gx: $0[4], gy: $0[5], gz: $0[6]) }
        let flow = readCSV(url.appendingPathComponent(CSVSchema.flowFile)).map {
            FlowSample(t: $0[0], vx: $0[1], vy: $0[2], quality: $0[3], h: $0[4]) }
        let gnss = readCSV(url.appendingPathComponent(CSVSchema.gnssFile)).map {
            GnssSample(t: $0[0], speed: $0[1], speedAcc: $0[2], course: $0[3]) }
        let events = readEvents(url.appendingPathComponent(CSVSchema.eventsFile))
        let est = readCSV(url.appendingPathComponent(CSVSchema.estFile))
        guard let filter = FilterRegistry.make(name: filterName, config: settings.filterConfig) else { continue }
        var rows: [EstRow] = []
        _ = Replay.run(filter: filter, imu: imu, flow: flow, gnss: gnss, events: events) { rows.append($0) }
        var maxDv = 0.0
        if rows.count == est.count {
            for (a, b) in zip(rows, est) {
                maxDv = max(maxDv, abs(a.vx - b[1]), abs(a.vy - b[2]))
            }
        } else {
            maxDv = .infinity
        }
        r.check("engine[\(filterName)]: replay of logged CSVs reproduces est.csv (≤1e-6 m/s)",
                maxDv <= 1e-6, "rows \(rows.count)/\(est.count), max |Δv| \(maxDv)")
    }
}

/// Distance is "since run start": it must survive back-to-back pushes (phone-sim loop
/// boundaries) while recording and reset only on start_run / reset_distance.
private func distanceContinuityChecks(_ r: inout CheckRunner, dir: URL) {
    let rt = GroundSpeedRuntime(settings: RuntimeSettings(), runsDirectory: dir)
    let push = SimPush(config: SimPush.Config())
    // Distance before a run (since launch) must not leak into the run.
    let pre = Clock.now()
    for s in push.samples { SimPush.feed(s, t0: pre, into: rt.engine) }
    rt.engine.drain()
    r.near("distance: accumulates before any run (since launch)", rt.engine.snapshot().distance, 10, tol: 0.15)

    _ = try? rt.startRun(label: "two_pushes")
    var t0 = pre + push.duration + 0.01
    for _ in 0..<2 {
        for s in push.samples { SimPush.feed(s, t0: t0, into: rt.engine) }
        t0 += push.duration + 0.01
    }
    rt.engine.drain()
    let during = rt.engine.snapshot().distance
    let frame = rt.makeFrame()
    r.near("distance: two back-to-back pushes in one run → 20 m (no reset at loop boundary)", during, 20, tol: 0.3)
    r.check("distance: telemetry frame carries the same distance", abs(Double(frame.distance) - during) < 1e-3,
            "frame \(frame.distance) vs engine \(during)")
    let reply = rt.handle(.stopRun)
    r.check("distance: stop_run reply carries run distance", reply.ok && abs((reply.distance ?? -1) - during) < 1e-6,
            "\(String(describing: reply.distance))")
    r.check("distance: stop_run JSON has \"distance\" key",
            String(decoding: reply.encodeLine(), as: UTF8.self).contains("\"distance\":"))
    let est = readCSV(dir.appendingPathComponent(reply.run ?? "").appendingPathComponent(CSVSchema.estFile))
    r.near("distance: est.csv final distance matches", est.last?[5] ?? -1, during, tol: 1e-3)

    rt.resetDistance()
    rt.engine.drain()
    r.near("distance: reset_distance zeroes it", rt.engine.snapshot().distance, 0, tol: 1e-9)
}

/// start_run during cruise must not disturb the estimate: only distance restarts.
private func startMidMotionChecks(_ r: inout CheckRunner, dir: URL) {
    let rt = GroundSpeedRuntime(settings: RuntimeSettings(), runsDirectory: dir)
    let push = SimPush(config: SimPush.Config())
    let vxs = LockedValues()
    rt.engine.onSnapshot = { vxs.append($0.vx) }
    let t0 = Clock.now()
    var started = false
    var startIndex = 0
    for s in push.samples {
        if !started, s.t >= 6.0 {          // cruising at 1 m/s, flow healthy
            rt.engine.drain()
            startIndex = vxs.count
            _ = try? rt.startRun(label: "mid_motion")
            started = true
        }
        SimPush.feed(s, t0: t0, into: rt.engine)
    }
    rt.engine.drain()
    let v = vxs.values
    var maxJump = 0.0
    for i in 1..<v.count { maxJump = max(maxJump, abs(v[i] - v[i - 1])) }
    let around = v[max(0, startIndex - 2)..<min(v.count, startIndex + 5)].map { String(format: "%.3f", $0) }
    r.check("start_run mid-cruise: v_x continuous (max |Δv_x| between steps < 0.1)", maxJump < 0.1,
            String(format: "max jump %.4f; around start: %@", maxJump, around.joined(separator: " ")))
    r.check("start_run mid-cruise: v_x stays near 1 m/s right after start",
            v[startIndex..<min(v.count, startIndex + 20)].allSatisfy { abs($0 - 1) < 0.1 })
    let dist = rt.engine.snapshot().distance
    let expected = 10 - SimPush.truth(push.config, 6.0).x
    r.near("start_run mid-cruise: run distance counts from start", dist, expected, tol: 0.15)
    _ = rt.stopRun()
}

final class LockedValues: @unchecked Sendable {
    private let lock = NSLock()
    private var v: [Double] = []
    func append(_ x: Double) { lock.withLock { v.append(x) } }
    var values: [Double] { lock.withLock { v } }
    var count: Int { lock.withLock { v.count } }
}

private func readCSV(_ url: URL) -> [[Double]] {
    guard let text = try? String(contentsOf: url, encoding: .utf8) else { return [] }
    return text.split(separator: "\n").dropFirst().map { line in
        CSVRow.split(line).map { Double($0) ?? .nan }
    }
}

private func readEvents(_ url: URL) -> [RunEvent] {
    guard let text = try? String(contentsOf: url, encoding: .utf8) else { return [] }
    return text.split(separator: "\n").dropFirst().compactMap { line in
        let c = CSVRow.split(line)
        guard c.count >= 2, let t = Double(c[0]) else { return nil }
        return RunEvent(t: t, event: c[1], value: c.count > 2 ? c[2] : "")
    }
}

// MARK: - UDP sender → local socket

private func senderRoundtripChecks(_ r: inout CheckRunner) {
    guard let sock = UDPTestSocket() else { r.check("udp: bind local socket", false); return }
    defer { sock.close() }
    let sender = TelemetrySender(host: "127.0.0.1", port: sock.port, rateHz: 50)
    let frame = TelemetryFrame(t: 1234.5, vx: 1.25, vy: -0.03, sigmaVx: 0.05, sigmaVy: 0.06, distance: 3.5,
                               flowQuality: 12, h: 0.3, status: [.imuOK, .flowOK, .filterInit],
                               battery: 87, torch: 60)
    sender.frameSource = { frame }
    sender.start()
    var got: [TelemetryFrame] = []
    let deadline = Date().addingTimeInterval(2)
    while got.count < 5, Date() < deadline {
        if let d = sock.receive(timeout: 0.5), let f = try? TelemetryFrame.decode(d) { got.append(f) }
    }
    r.check("udp: binary frames received and decoded (CRC ok)", got.count >= 5, "\(got.count) frames")
    if let f = got.first {
        var expect = frame
        expect.seq = f.seq
        r.check("udp: decoded frame equals sent frame", f == expect)
        r.check("udp: seq increments by 1", zip(got, got.dropFirst()).allSatisfy { $1.seq == $0.seq &+ 1 })
    }
    sender.useJSON = true
    var json: Data?
    let d2 = Date().addingTimeInterval(2)
    while json == nil, Date() < d2 {
        if let d = sock.receive(timeout: 0.5), d.first == UInt8(ascii: "{") { json = d }
    }
    let jf = json.flatMap { try? TelemetryFrame.decode($0) }
    r.check("udp: JSON debug frame decodes", jf?.vx == 1.25 && jf?.battery == 87)
    sender.stop()
}

/// Minimal blocking BSD UDP socket on 127.0.0.1:<ephemeral>.
private final class UDPTestSocket {
    let fd: Int32
    let port: UInt16

    init?() {
        let fd = socket(AF_INET, SOCK_DGRAM, 0)
        guard fd >= 0 else { return nil }
        var addr = sockaddr_in()
        addr.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = 0
        addr.sin_addr.s_addr = inet_addr("127.0.0.1")
        let ok = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                bind(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) == 0
            }
        }
        guard ok else { Darwin.close(fd); return nil }
        var len = socklen_t(MemoryLayout<sockaddr_in>.size)
        _ = withUnsafeMutablePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { getsockname(fd, $0, &len) }
        }
        self.fd = fd
        self.port = UInt16(bigEndian: addr.sin_port)
    }

    func receive(timeout: Double) -> Data? {
        var tv = timeval(tv_sec: Int(timeout), tv_usec: Int32((timeout - floor(timeout)) * 1e6))
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        var buf = [UInt8](repeating: 0, count: 2048)
        let n = recv(fd, &buf, buf.count, 0)
        return n > 0 ? Data(buf[0..<n]) : nil
    }

    func close() { Darwin.close(fd) }
}

// MARK: - TCP command server

private func commandServerChecks(_ r: inout CheckRunner, dir: URL) {
    var settings = RuntimeSettings()
    settings.commandPort = UInt16(39_000 + Int(ProcessInfo.processInfo.processIdentifier % 2000))
    settings.dashboardHost = "10.255.255.1"
    let rt = GroundSpeedRuntime(settings: settings, runsDirectory: dir, bonjourName: "gsk-checks")
    do { try rt.server.start() } catch { r.check("tcp: server starts", false, "\(error)"); return }
    defer { rt.server.stop(); rt.sender.stop() }
    r.check("tcp: server listening", rt.server.waitUntilReady(timeout: 3), rt.server.state)
    guard let client = TCPTestClient(port: settings.commandPort) else {
        r.check("tcp: connect to 127.0.0.1", false); return
    }
    defer { client.close() }

    func ask(_ line: String) -> CommandReply? {
        client.send(line + "\n")
        return client.readLine(timeout: 3).flatMap { try? CommandReply.parse(Data($0.utf8)) }
    }
    let ping = ask(#"{"cmd":"ping"}"#)
    r.check("tcp: ping → ok reply", ping?.ok == true && ping?.cmd == "ping", "\(String(describing: ping))")

    // Peer adoption: the sender should now target the client's IP.
    let d = Date().addingTimeInterval(1)
    while rt.sender.host != "127.0.0.1", Date() < d { Thread.sleep(forTimeInterval: 0.02) }
    r.check("tcp: connecting peer becomes UDP destination", rt.sender.host == "127.0.0.1", rt.sender.host)

    let start = ask(#"{"cmd":"start_run","label":"tcp_test"}"#)
    let runId = start?.run ?? ""
    r.check("tcp: start_run → ok with run id", start?.ok == true && runId.hasSuffix("_tcp_test"), "\(String(describing: start))")
    r.check("tcp: start_run created run folder",
            FileManager.default.fileExists(atPath: dir.appendingPathComponent(runId).appendingPathComponent(CSVSchema.imuFile).path))
    let mark = ask(#"{"cmd":"mark","label":"lens_covered"}"#)
    r.check("tcp: mark → ok", mark?.ok == true)
    let torch = ask(#"{"cmd":"set_torch","level":0.6}"#)
    r.check("tcp: set_torch → ok, level applied", torch?.ok == true && abs(rt.torchLevel - 0.6) < 1e-9)
    let bad = ask(#"{"cmd":"warp_drive"}"#)
    r.check("tcp: unknown command → ok:false with cmd name", bad?.ok == false && bad?.cmd == "warp_drive", "\(String(describing: bad))")
    let garbage = ask("not json")
    r.check("tcp: garbage line → ok:false", garbage?.ok == false)
    let stop = ask(#"{"cmd":"stop_run"}"#)
    r.check("tcp: stop_run → ok with same run id", stop?.ok == true && stop?.run == runId)
    r.check("tcp: meta.json written on stop",
            FileManager.default.fileExists(atPath: dir.appendingPathComponent(runId).appendingPathComponent(CSVSchema.metaFile).path))
    let events = (try? String(contentsOf: dir.appendingPathComponent(runId).appendingPathComponent(CSVSchema.eventsFile), encoding: .utf8)) ?? ""
    r.check("tcp: events.csv has start/mark/torch/stop",
            ["start", "mark", "torch", "stop"].allSatisfy { events.contains(",\($0),") }, events.replacingOccurrences(of: "\n", with: " | "))
    let stop2 = ask(#"{"cmd":"stop_run"}"#)
    r.check("tcp: stop_run when idle → ok:false", stop2?.ok == false)
}

/// Minimal blocking BSD TCP client.
private final class TCPTestClient {
    let fd: Int32
    private var buffer = Data()

    init?(port: UInt16) {
        let fd = socket(AF_INET, SOCK_STREAM, 0)
        guard fd >= 0 else { return nil }
        var addr = sockaddr_in()
        addr.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = port.bigEndian
        addr.sin_addr.s_addr = inet_addr("127.0.0.1")
        let ok = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                connect(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) == 0
            }
        }
        guard ok else { Darwin.close(fd); return nil }
        self.fd = fd
    }

    func send(_ s: String) {
        let bytes = Array(s.utf8)
        _ = bytes.withUnsafeBytes { Darwin.send(fd, $0.baseAddress, bytes.count, 0) }
    }

    func readLine(timeout: Double) -> String? {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if let nl = buffer.firstIndex(of: 0x0A) {
                let line = buffer[buffer.startIndex..<nl]
                buffer = Data(buffer[(nl + 1)...])
                return String(decoding: line, as: UTF8.self)
            }
            var tv = timeval(tv_sec: 0, tv_usec: 200_000)
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
            var buf = [UInt8](repeating: 0, count: 4096)
            let n = recv(fd, &buf, buf.count, 0)
            if n > 0 { buffer.append(contentsOf: buf[0..<n]) } else if n == 0 { return nil }
        }
        return nil
    }

    func close() { Darwin.close(fd) }
}
