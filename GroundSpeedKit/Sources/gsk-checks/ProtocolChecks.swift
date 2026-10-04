import Foundation
import SpeedProtocol

let goldenHex = "a5012a00000000000000004a93400000a03f8fc2f5bccdcc4c3d8fc2753d00006040000040419a99993e0302573c3ccd"

func goldenFrame() -> TelemetryFrame {
    TelemetryFrame(seq: 42, t: 1234.5, vx: 1.25, vy: -0.03, sigmaVx: 0.05, sigmaVy: 0.06,
                   distance: 3.5, flowQuality: 12.0, h: 0.3, status: StatusFlags(rawValue: 0x0203),
                   battery: 87, torch: 60, version: 1)
}

func hex(_ d: Data) -> String { d.map { String(format: "%02x", $0) }.joined() }

func jsonKeys(_ d: Data) -> Set<String> {
    guard let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any] else { return [] }
    return Set(o.keys)
}

func protocolChecks(_ r: inout CheckRunner) {
    r.section("SpeedProtocol")
    r.check("CRC16 check value 0x29B1", CRC16.ccittFalse(Array("123456789".utf8)) == 0x29B1)

    let f = goldenFrame()
    let bin = f.encodeBinary()
    r.check("binary frame is 48 bytes", bin.count == 48, "\(bin.count)")
    r.check("binary frame matches docs/golden_frame.md", hex(bin) == goldenHex, hex(bin))
    var f2 = f; f2.version = 2; f2.ax = 0.5; f2.ay = -0.25; f2.gz = 0.1; f2.flowVx = 1.2; f2.flowVy = -0.1; f2.netForward = 3.25
    let bin2 = f2.encodeBinary()
    r.check("v2 binary frame is 72 bytes", bin2.count == 72, "\(bin2.count)")
    r.check("v2 binary roundtrip", (try? TelemetryFrame.decode(bin2)) == f2)
    r.check("v2 JSON roundtrip", (try? TelemetryFrame.decode(f2.encodeJSON())) == f2)
    r.check("v2 shares v1 prefix (bytes 2..45)", Array(bin2[2..<46]) == Array(bin[2..<46]))
    r.check("status flags 0x0203 = IMU_OK|FLOW_OK|FILTER_INIT",
            StatusFlags([.imuOK, .flowOK, .filterInit]).rawValue == 0x0203)
    r.check("calibrating is bit 10", StatusFlags.calibrating.rawValue == 1 << 10)

    var rt: TelemetryFrame?
    r.noThrow("binary decode") { rt = try TelemetryFrame.decode(bin) }
    r.check("binary encode→decode roundtrip", rt == f)

    var bad = bin; bad[20] ^= 0x01
    var crcRejected = false
    do { _ = try TelemetryFrame.decode(bad) } catch TelemetryDecodeError.badCRC { crcRejected = true } catch {}
    r.check("corrupted payload rejected (CRC)", crcRejected)
    var badCrc = bin; badCrc[47] ^= 0xFF
    r.check("corrupted CRC bytes rejected", (try? TelemetryFrame.decode(badCrc)) == nil)
    var badMagic = bin; badMagic[0] = 0x5A
    var magicRejected = false
    do { _ = try TelemetryFrame.decode(badMagic) } catch TelemetryDecodeError.badMagic { magicRejected = true } catch {}
    r.check("bad magic rejected", magicRejected)
    r.check("short frame rejected", (try? TelemetryFrame.decode(bin.prefix(47))) == nil)

    let json = f.encodeJSON()
    let expectedKeys: Set<String> = ["seq", "t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance",
                                     "flow_quality", "h", "status", "battery", "torch"]
    r.check("JSON frame has exactly the protocol keys", jsonKeys(json) == expectedKeys,
            String(decoding: json, as: UTF8.self))
    r.check("JSON roundtrip via decode() ('{' detection)", (try? TelemetryFrame.decode(json)) == f)
    let pyJSON = #"{"seq":7,"t":10.25,"v_x":0.5,"v_y":0,"sigma_vx":0.1,"sigma_vy":0.1,"distance":1,"flow_quality":0,"h":0.3,"status":3,"battery":255,"torch":0}"#
    let pf = try? TelemetryFrame.decode(Data(pyJSON.utf8))
    r.check("JSON decode of hand-written frame", pf?.seq == 7 && pf?.battery == 255 && pf?.status == [.imuOK, .flowOK])

    // Commands
    let cases: [(String, Command)] = [
        (#"{"cmd":"ping"}"#, .ping),
        (#"{"cmd":"start_run"}"#, .startRun(label: nil)),
        (#"{"cmd":"start_run","label":"steady_carpet_03"}"#, .startRun(label: "steady_carpet_03")),
        (#"{"cmd":"stop_run"}"#, .stopRun),
        (#"{"cmd":"mark"}"#, .mark(label: nil)),
        (#"{"cmd":"mark","label":"lens_covered"}"#, .mark(label: "lens_covered")),
        (#"{"cmd":"calibrate"}"#, .calibrate),
        (#"{"cmd":"set_torch","level":0.6}"#, .setTorch(level: 0.6)),
        (#"{"cmd":"reset_distance"}"#, .resetDistance),
    ]
    var parseOK = true, encodeOK = true
    for (line, cmd) in cases {
        if (try? Command.parse(line + "\n")) != cmd { parseOK = false; print("   parse mismatch: \(line)") }
        let enc = cmd.encodeLine()
        if enc.last != 0x0A || (try? Command.parse(enc)) != cmd { encodeOK = false }
        let encObj = try? JSONSerialization.jsonObject(with: enc) as? NSDictionary
        let expObj = try? JSONSerialization.jsonObject(with: Data(line.utf8)) as? NSDictionary
        if encObj != expObj { encodeOK = false; print("   encode mismatch: \(String(decoding: enc, as: UTF8.self))") }
    }
    r.check("all 7 commands parse", parseOK)
    r.check("commands encode to protocol JSON + newline", encodeOK)
    var unknown = false
    do { _ = try Command.parse(#"{"cmd":"selfdestruct"}"#) } catch Command.ParseError.unknownCommand { unknown = true } catch {}
    r.check("unknown command rejected", unknown)
    r.check("set_torch without level rejected", (try? Command.parse(#"{"cmd":"set_torch"}"#)) == nil)
    r.check("garbage line rejected", (try? Command.parse("not json")) == nil)

    let okReply = CommandReply.success("start_run", run: "2026-10-03_18-40-12")
    r.check("reply ok keys", jsonKeys(okReply.encodeLine()) == ["ok", "cmd", "run"])
    let errReply = CommandReply.failure("stop_run", "not recording")
    r.check("reply error keys", jsonKeys(errReply.encodeLine()) == ["ok", "cmd", "error"])
    r.check("reply roundtrip", (try? CommandReply.parse(errReply.encodeLine())) == errReply)

    var lb = LineBuffer()
    let a = lb.append(Data(#"{"cmd":"pi"#.utf8))
    let b = lb.append(Data("ng\"}\r\n{\"cmd\":\"stop_run\"}\n\n".utf8))
    r.check("LineBuffer splits partial/CRLF lines", a.isEmpty && b.count == 2
            && (try? Command.parse(b[0])) == .ping && (try? Command.parse(b[1])) == .stopRun)

    // CSV
    r.check("CSV headers frozen",
            CSVSchema.imuHeader == "t,ax,ay,az,gx,gy,gz"
            && CSVSchema.flowHeader == "t,vx_cam,vy_cam,quality,h"
            && CSVSchema.gnssHeader == "t,speed,speed_acc,course"
            && CSVSchema.depthHeader == "t,h"
            && CSVSchema.estHeader == "t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h"
            && CSVSchema.eventsHeader == "t,event,value")
    let row = CSVRow.format(t: 123456.1234567, [1.0 / 3.0, -2.5])
    r.check("CSVRow t microsecond + %.9g", row == "123456.123457,0.333333333,-2.5", row)
    r.check("CSV text quoting roundtrip", CSVRow.split(Substring(CSVRow.join(["1.0", CSVRow.text("a,\"b\""), "x"]))) == ["1.0", "a,\"b\"", "x"])

    let meta = RunMeta(runId: "2026-10-03_18-40-12_test", label: "test", filterName: "ReferenceKF4",
                       mountHeightM: 0.3, focalPx: 1400, mountConfig: .object(["swap_xy": .bool(false)]),
                       startT: 1000, q: ["q_accel": 0.02], r: ["r_flow_base": 9e-4])
    let mj = meta.encodedJSON()
    let metaKeys: Set<String> = ["run_id", "label", "app_version", "device", "filter_name", "mount_height_m",
                                 "torch_level", "scale_factor", "focal_px", "mount_config", "start_t", "end_t",
                                 "battery_start", "battery_end", "q", "r"]
    r.check("meta.json keys", jsonKeys(mj) == metaKeys)
    r.check("meta.json roundtrip", (try? JSONDecoder().decode(RunMeta.self, from: mj)) == meta)
}
