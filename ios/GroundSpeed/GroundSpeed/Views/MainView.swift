import PhoneRuntime
import SpeedProtocol
import SwiftUI

/// The main screen, laid out like a datasheet: distance as the hero, velocities with ±σ,
/// one status row, one quiet line of diagnostics, a calm Record key and Mark.
/// Everything else (torch, setup, runs) lives behind the sheet.
struct MainView: View {
    @EnvironmentObject var model: AppModel
    @State private var appeared = false
    @State private var showSheet = false

    var body: some View {
        ZStack {
            Theme.paper.ignoresSafeArea()
            VStack(alignment: .leading, spacing: 0) {
                TitleRow(showSheet: $showSheet)
                    .reveal(0, appeared)
                Spacer(minLength: 24)
                DistanceBlock()
                    .reveal(1, appeared)
                Spacer(minLength: 24)
                Hairline()
                VelocityBlock()
                    .padding(.vertical, 20)
                    .reveal(2, appeared)
                Hairline()
                StatusRow()
                    .padding(.top, 18)
                    .reveal(3, appeared)
                DiagnosticsLine()
                    .padding(.top, 10)
                    .reveal(3, appeared)
                if let err = model.errorMessage {
                    Text(err)
                        .font(Theme.mono(10))
                        .foregroundStyle(Theme.alert)
                        .padding(.top, 8)
                        .onTapGesture { model.errorMessage = nil }
                }
                Spacer(minLength: 24)
                ControlsRow()
                    .reveal(4, appeared)
            }
            .padding(.horizontal, 28)
            .padding(.vertical, 16)
        }
        .onAppear { appeared = true }
        .sheet(isPresented: $showSheet) {
            ConsoleSheet().environmentObject(model)
        }
    }
}

// MARK: - Title

private struct TitleRow: View {
    @EnvironmentObject var model: AppModel
    @Binding var showSheet: Bool

    var body: some View {
        HStack(alignment: .firstTextBaseline) {
            VStack(alignment: .leading, spacing: 4) {
                Text("Ground Speed")
                    .font(Theme.serif(22))
                    .foregroundStyle(Theme.ink)
                Micro(model.isRecording ? "rec · \(model.runId ?? "")" : model.readout.source)
                if model.readout.fromFPGA {
                    Micro("Kalman filter running on the FPGA", color: Theme.live)
                }
            }
            Spacer()
            Button {
                showSheet = true
            } label: {
                Image(systemName: "slider.horizontal.3")
                    .font(.system(size: 17, weight: .regular))
                    .foregroundStyle(Theme.ink)
                    .frame(width: 44, height: 44)
                    .contentShape(Rectangle())
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Console")
        }
    }
}

// MARK: - Distance (hero)

private struct DistanceBlock: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Micro(model.isRecording ? "Distance · this run" : "Distance")
            HStack(alignment: .firstTextBaseline, spacing: 8) {
                Text(String(format: "%.2f", model.readout.distance))
                    .font(Theme.mono(88))
                    .foregroundStyle(Theme.ink)
                    .lineLimit(1)
                    .minimumScaleFactor(0.5)
                    .contentTransition(.numericText(value: model.readout.distance))
                    .animation(.easeOut(duration: 0.15), value: model.readout.distance)
                Text("m")
                    .font(Theme.serif(22))
                    .foregroundStyle(Theme.inkSoft)
            }
            SpeedLine(speed: model.readout.speed)
        }
    }
}

/// Ground speed |v| in m/s, with km/h and mph.
private struct SpeedLine: View {
    let speed: Double

    var body: some View {
        HStack(alignment: .firstTextBaseline, spacing: 6) {
            Micro("Speed")
            Text(String(format: "%.2f", speed))
                .font(Theme.mono(30))
                .foregroundStyle(Theme.ink)
                .contentTransition(.numericText(value: speed))
                .animation(.easeOut(duration: 0.15), value: speed)
            Text("m/s").font(Theme.serif(13)).foregroundStyle(Theme.inkSoft)
            Text(String(format: "· %.1f mph", speed * 2.236936))
                .font(Theme.mono(18))
                .foregroundStyle(Theme.ink)
            Text(String(format: "%.1f km/h", speed * 3.6))
                .font(Theme.mono(11))
                .foregroundStyle(Theme.inkSoft)
        }
    }
}

// MARK: - Velocities

private struct VelocityBlock: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        let r = model.readout
        HStack(alignment: .top) {
            VelocityCell(label: "v_x · forward", value: r.vx, sigma: r.sigmaVx)
            Spacer()
            VelocityCell(label: "v_y · left", value: r.vy, sigma: r.sigmaVy)
        }
    }
}

private struct VelocityCell: View {
    let label: String
    let value: Double
    let sigma: Double

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Micro(label)
            HStack(alignment: .firstTextBaseline, spacing: 4) {
                Text(String(format: "%+.2f", value))
                    .font(Theme.mono(34))
                    .foregroundStyle(Theme.ink)
                    .contentTransition(.numericText(value: value))
                    .animation(.easeOut(duration: 0.15), value: value)
                Text("m/s").font(Theme.serif(13)).foregroundStyle(Theme.inkSoft)
            }
            Text(String(format: "± %.3f", sigma))
                .font(Theme.mono(12))
                .foregroundStyle(Theme.inkSoft)
        }
    }
}

// MARK: - Status

private struct StatusRow: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        let st = model.snap.status
        HStack(spacing: 16) {
            StatusDot(name: "IMU", state: st.contains(.imuOK) ? .ok : .fault)
            StatusDot(name: "FLOW", state: st.contains(.flowOK) ? .ok : .fault)
            StatusDot(name: "GNSS", state: st.contains(.gnssOK) ? .ok : .idle)
            StatusDot(name: "LiDAR", state: st.contains(.lidarOK) ? .ok : .idle)
            StatusDot(name: "LINK", state: model.dashboardClients > 0 ? .ok : .idle)
            StatusDot(name: "FPGA", state: model.readout.fromFPGA ? .ok : .idle)
            Spacer(minLength: 0)
        }
    }
}

/// ok = filled teal; fault = filled vermilion; idle = hollow (not expected / optional).
struct StatusDot: View {
    enum Level { case ok, fault, idle }
    let name: String
    let state: Level

    private var color: Color {
        switch state {
        case .ok: return Theme.live
        case .fault: return Theme.alert
        case .idle: return Theme.inkFaint
        }
    }

    var body: some View {
        HStack(spacing: 5) {
            ZStack {
                Circle().stroke(color, lineWidth: 1)
                Circle().fill(state == .idle ? Color.clear : color)
            }
            .frame(width: 7, height: 7)
            Text(name)
                .font(.custom("Menlo-Regular", size: 10, relativeTo: .caption2))
                .tracking(0.8)
                .foregroundStyle(state == .fault ? Theme.alert : Theme.inkSoft)
        }
        .animation(.easeInOut(duration: 0.25), value: state)
        .accessibilityElement(children: .combine)
    }
}

private struct DiagnosticsLine: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        let s = model.snap
        // Speed accuracy is what the filter uses; horizontal (position) accuracy of a phone is
        // ±3–10 m no matter what and doesn't enter the speed estimate.
        let gnss = model.gnssSpeedAcc >= 0
            ? String(format: "±%.2f m/s (pos ±%.0f m)", model.gnssSpeedAcc, model.gnssHorizontalAcc)
            : "—"
        let bat = model.battery.map { "\($0)%" } ?? "—"
        let hSource = s.status.contains(.lidarOK) ? "LiDAR" : "set"
        // While the FPGA's numbers are on screen, keep the phone filter's speed for comparison.
        let phoneKF = model.readout.fromFPGA ? String(format: " · PHONE KF %.2f m/s", s.speed) : ""
        Text(String(format: "PSR %.1f · h %.3f m %@ · GNSS %@ · BAT %@", s.flowQuality, s.h, hSource, gnss, bat) + phoneKF)
            .font(Theme.mono(10))
            .foregroundStyle(Theme.inkSoft)
            .lineLimit(1)
            .minimumScaleFactor(0.7)
    }
}

// MARK: - Controls

private struct ControlsRow: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        HStack(alignment: .center) {
            RecordKey()
            Spacer()
            VStack(alignment: .trailing, spacing: 6) {
                Button {
                    model.zero()
                } label: {
                    Text("Zero")
                        .font(Theme.serif(17))
                        .foregroundStyle(Theme.ink)
                        .frame(width: 112, height: 40)
                        .overlay(Capsule().stroke(Theme.ink, lineWidth: 1))
                        .contentShape(Capsule())
                }
                .buttonStyle(.plain)
                .sensoryFeedback(.impact(weight: .light), trigger: model.zeros)
                Button {
                    model.mark()
                } label: {
                    Text("Mark")
                        .font(Theme.serif(17))
                        .foregroundStyle(Theme.ink)
                        .frame(width: 112, height: 48)
                        .overlay(Capsule().stroke(Theme.ink, lineWidth: 1))
                        .contentShape(Capsule())
                }
                .buttonStyle(.plain)
                .sensoryFeedback(.impact(weight: .light), trigger: model.marks)
                Micro(model.marks > 0 ? "\(model.marks) marks" : "event marker")
            }
        }
    }
}

/// Large but calm: a filled graphite disc; a vermilion ring while recording.
private struct RecordKey: View {
    @EnvironmentObject var model: AppModel

    var body: some View {
        HStack(spacing: 16) {
            Button {
                model.toggleRecording()
            } label: {
                ZStack {
                    Circle()
                        .stroke(Theme.alert, lineWidth: 3)
                        .frame(width: 104, height: 104)
                        .opacity(model.isRecording ? 1 : 0)
                    Circle()
                        .fill(Theme.ink)
                        .frame(width: 88, height: 88)
                    if model.isRecording {
                        RoundedRectangle(cornerRadius: 3)
                            .fill(Theme.alert)
                            .frame(width: 24, height: 24)
                    } else {
                        Circle()
                            .fill(Theme.alert)
                            .frame(width: 22, height: 22)
                    }
                }
                .frame(width: 108, height: 108)
                .contentShape(Circle())
                .animation(.easeInOut(duration: 0.25), value: model.isRecording)
            }
            .buttonStyle(.plain)
            .sensoryFeedback(.impact(weight: .medium), trigger: model.isRecording)
            .accessibilityLabel(model.isRecording ? "Stop recording" : "Start recording")

            VStack(alignment: .leading, spacing: 4) {
                Micro(model.isRecording ? "Recording" : "Record", color: model.isRecording ? Theme.alert : Theme.inkSoft)
                if model.isRecording, let start = model.runStart {
                    TimelineView(.periodic(from: start, by: 1)) { ctx in
                        Text(Self.clock(ctx.date.timeIntervalSince(start)))
                            .font(Theme.mono(15))
                            .foregroundStyle(Theme.ink)
                    }
                } else {
                    Text("00:00").font(Theme.mono(15)).foregroundStyle(Theme.inkFaint)
                }
            }
        }
    }

    static func clock(_ t: TimeInterval) -> String {
        String(format: "%02d:%02d", Int(t) / 60, Int(t) % 60)
    }
}
