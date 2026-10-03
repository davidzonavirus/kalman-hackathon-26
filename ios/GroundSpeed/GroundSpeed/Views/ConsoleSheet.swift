import KalmanCore
import OpticalFlow
import PhoneRuntime
import SwiftUI

/// Everything that isn't glanceable: instrument controls, setup, and the run log.
struct ConsoleSheet: View {
    enum Pane: String, CaseIterable, Identifiable {
        case controls = "Controls", setup = "Setup", runs = "Runs"
        var id: String { rawValue }
    }

    @EnvironmentObject var model: AppModel
    @Environment(\.dismiss) private var dismiss
    @State private var tab: Pane = .controls

    var body: some View {
        VStack(spacing: 0) {
            HStack(alignment: .firstTextBaseline) {
                Text("Console").font(Theme.serif(20)).foregroundStyle(Theme.ink)
                Spacer()
                Button {
                    dismiss()
                } label: {
                    Text("Done").font(Theme.serif(16)).foregroundStyle(Theme.ink)
                        .frame(minWidth: 44, minHeight: 44)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
            }
            .padding(.horizontal, 24)
            .padding(.top, 20)

            Picker("Section", selection: $tab) {
                ForEach(Pane.allCases) { t in Text(t.rawValue).tag(t) }
            }
            .pickerStyle(.segmented)
            .padding(.horizontal, 24)
            .padding(.vertical, 14)

            switch tab {
            case .controls: ControlsPane()
            case .setup: SetupPane()
            case .runs: RunsPane()
            }
        }
        .background(Theme.paper.ignoresSafeArea())
        .presentationDragIndicator(.visible)
    }
}

// MARK: - Building blocks

/// Section heading: micro label over a hairline.
struct SectionHead: View {
    let title: String
    init(_ title: String) { self.title = title }
    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Micro(title)
            Hairline()
        }
        .padding(.top, 18)
    }
}

/// Label left (serif), value/control right.
struct Row<Content: View>: View {
    let label: String
    @ViewBuilder var content: Content
    var body: some View {
        HStack(alignment: .firstTextBaseline) {
            Text(label).font(Theme.serif(15)).foregroundStyle(Theme.ink)
            Spacer(minLength: 12)
            content
        }
        .frame(minHeight: 34)
    }
}

struct MonoValue: View {
    let text: String
    var color: Color = Theme.ink
    var body: some View {
        Text(text).font(Theme.mono(13)).foregroundStyle(color)
    }
}

struct NumberField: View {
    @Binding var value: Double
    var body: some View {
        TextField("0", value: $value, format: .number.precision(.significantDigits(1...6)))
            .keyboardType(.numbersAndPunctuation)
            .multilineTextAlignment(.trailing)
            .font(Theme.mono(14))
            .foregroundStyle(Theme.ink)
            .frame(maxWidth: 130)
    }
}

struct SwitchRow: View {
    let label: String
    @Binding var isOn: Bool
    var body: some View {
        Toggle(isOn: $isOn) {
            Text(label).font(Theme.serif(15)).foregroundStyle(Theme.ink)
        }
        .tint(Theme.live)
        .frame(minHeight: 34)
    }
}

struct QuietButton: View {
    let title: String
    var role: Color = Theme.ink
    let action: () -> Void
    var body: some View {
        Button(action: action) {
            Text(title)
                .font(Theme.serif(14))
                .foregroundStyle(role)
                .padding(.horizontal, 14).padding(.vertical, 7)
                .overlay(Capsule().stroke(role.opacity(0.6), lineWidth: 1))
                .contentShape(Capsule())
        }
        .buttonStyle(.plain)
    }
}

struct Note: View {
    let text: String
    var color: Color = Theme.inkSoft
    init(_ text: String, color: Color = Theme.inkSoft) { self.text = text; self.color = color }
    var body: some View {
        Text(text).font(Theme.serif(12)).foregroundStyle(color).fixedSize(horizontal: false, vertical: true)
    }
}

// MARK: - Controls

private struct ControlsPane: View {
    @EnvironmentObject var model: AppModel
    @State private var torch: Double = 0.5
    @State private var editingTorch = false

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 0) {
                SectionHead("Torch")
                Row(label: "Level") {
                    MonoValue(text: "\(Int((torch * 100).rounded()))%")
                }
                Slider(value: $torch, in: 0...1, step: 0.05, onEditingChanged: { editing in
                    editingTorch = editing
                    if !editing { model.setTorch(torch) }
                })
                .tint(Theme.ink)

                SectionHead("Sensors")
                SwitchRow(label: "Sensors running", isOn: Binding(get: { model.sensorsOn },
                                                                  set: { _ in model.toggleSensors() }))
                Row(label: "Camera") {
                    MonoValue(text: model.camera.running ? String(format: "%.0f fps", model.camera.fps) : "off",
                              color: model.camera.running ? Theme.ink : Theme.inkFaint)
                }
                Note(model.camera.formatDescription
                     + (model.camera.focusLocked ? " · focus locked" : "")
                     + (model.camera.exposureLocked ? " · exposure ≤ 1 ms" : ""))
                if let e = model.camera.error { Note("Camera: \(e)", color: Theme.alert) }
                Row(label: "Rates") {
                    MonoValue(text: String(format: "IMU %.0f · flow %.0f Hz · %.1f ms",
                                           model.snap.imuRateHz, model.snap.flowRateHz, model.camera.processingMs))
                }

                SectionHead("Actions")
                Row(label: "Zero distance") { QuietButton(title: "Zero") { model.resetDistance() } }
                Row(label: "Calibrate IMU bias") {
                    QuietButton(title: model.snap.status.contains(.calibrating) ? "Calibrating…" : "Calibrate") {
                        model.calibrate()
                    }
                }
                Note("Cart must be still for 2 s.")
                Row(label: "Measure height (LiDAR)") {
                    QuietButton(title: model.measuringHeight ? "Measuring…" : "Measure") { model.measureHeight() }
                }
                Note(model.lidarMessage)

                SectionHead("Link")
                Row(label: "Telemetry") { MonoValue(text: "UDP \(model.telemetryTarget)") }
                Row(label: "Dashboard") {
                    MonoValue(text: model.dashboardClients > 0 ? (model.dashboardPeer ?? "connected") : "none",
                              color: model.dashboardClients > 0 ? Theme.live : Theme.inkFaint)
                }
                Row(label: "Frames sent") { MonoValue(text: "\(model.framesSent)") }
                Note("Command server: \(model.serverState)")
            }
            .padding(.horizontal, 24)
            .padding(.bottom, 40)
        }
        .onAppear { torch = model.settings.torchLevel }
        .onChange(of: model.settings.torchLevel) { _, v in if !editingTorch { torch = v } }
    }
}

// MARK: - Setup

private struct SetupPane: View {
    @EnvironmentObject var model: AppModel
    @State private var draft = RuntimeSettings()
    @State private var loaded = false

    var body: some View {
        VStack(spacing: 0) {
            ScrollView {
                VStack(alignment: .leading, spacing: 0) {
                    link
                    mount
                    orientation
                    filter
                    flow
                }
                .padding(.horizontal, 24)
                .padding(.bottom, 24)
            }
            Hairline()
            HStack {
                Note(draft == model.settings ? "No changes." : "Unsaved changes. Filter changes reset the filter.")
                Spacer()
                QuietButton(title: "Revert", role: Theme.inkSoft) { draft = model.settings }
                    .disabled(draft == model.settings)
                QuietButton(title: "Apply", role: draft == model.settings ? Theme.inkFaint : Theme.live) {
                    model.apply(draft)
                }
                .disabled(draft == model.settings)
            }
            .padding(.horizontal, 24)
            .padding(.vertical, 12)
        }
        .onAppear {
            if !loaded { draft = model.settings; loaded = true }
        }
        .onChange(of: model.settings) { _, new in
            // Values changed elsewhere (LiDAR height, learned mapping, torch): pick them up.
            draft.manualHeight = new.manualHeight
            draft.flowMapping = new.flowMapping
            draft.torchLevel = new.torchLevel
        }
    }

    private var link: some View {
        VStack(alignment: .leading, spacing: 0) {
            SectionHead("Dashboard link")
            Row(label: "Dashboard IP") {
                TextField("172.20.10.2", text: $draft.dashboardHost)
                    .keyboardType(.numbersAndPunctuation)
                    .textInputAutocapitalization(.never)
                    .autocorrectionDisabled()
                    .multilineTextAlignment(.trailing)
                    .font(Theme.mono(14))
                    .foregroundStyle(Theme.ink)
            }
            SwitchRow(label: "JSON debug frames", isOn: $draft.useJSON)
            Note("A dashboard that connects on TCP 9001 becomes the UDP destination automatically.")
        }
    }

    private var mount: some View {
        VStack(alignment: .leading, spacing: 0) {
            SectionHead("Mount")
            Row(label: "Camera height h (m)") { NumberField(value: $draft.manualHeight) }
            Note("Used when LiDAR isn't live. Measure it from Controls › Measure height.")
        }
    }

    private var orientation: some View {
        VStack(alignment: .leading, spacing: 0) {
            SectionHead("Orientation check")
            Note("Push the cart forward: flow v_x and IMU a_x must go positive. If not, toggle the mapping or learn it from a push.")
            HStack {
                LiveValue(label: "flow v_x", value: model.snap.flowVx, threshold: 0.05)
                Spacer()
                LiveValue(label: "flow v_y", value: model.snap.flowVy, threshold: .infinity)
                Spacer()
                LiveValue(label: "imu a_x", value: model.snap.imuAx, threshold: 0.1)
            }
            .padding(.vertical, 10)
            Micro("Flow · camera → vehicle")
            SwitchRow(label: "Swap x / y", isOn: $draft.flowMapping.swapXY)
            SwitchRow(label: "Flip x", isOn: $draft.flowMapping.flipX)
            SwitchRow(label: "Flip y", isOn: $draft.flowMapping.flipY)
            if !draft.flowMapping.isProper {
                Note("Mirrored mapping: not physical for a downward camera.", color: Theme.alert)
            }
            Row(label: "Learn from a 3 s push") {
                QuietButton(title: "Learn") { model.learnMountFromPush() }
            }
            if let m = model.learnMessage { Note(m) }
            Micro("IMU · device → vehicle").padding(.top, 10)
            Note("Default: flat, screen up, top of phone forward.")
            SwitchRow(label: "Swap x / y", isOn: $draft.imuMapping.swapXY)
            SwitchRow(label: "Flip x", isOn: $draft.imuMapping.flipX)
            SwitchRow(label: "Flip y", isOn: $draft.imuMapping.flipY)
        }
    }

    private var filter: some View {
        VStack(alignment: .leading, spacing: 0) {
            SectionHead("Filter")
            Picker("Filter", selection: $draft.filterName) {
                ForEach(FilterRegistry.names, id: \.self) { name in Text(name).tag(name) }
            }
            .pickerStyle(.segmented)
            .padding(.vertical, 8)
            Group {
                Row(label: "q_accel") { NumberField(value: $draft.filterConfig.qAccel) }
                Row(label: "q_bias") { NumberField(value: $draft.filterConfig.qBias) }
                Row(label: "r_flow_base") { NumberField(value: $draft.filterConfig.rFlowBase) }
                Row(label: "psr_ref") { NumberField(value: $draft.filterConfig.psrRef) }
                Row(label: "psr_min") { NumberField(value: $draft.filterConfig.psrMin) }
            }
            Group {
                Row(label: "r_zupt") { NumberField(value: $draft.filterConfig.rZupt) }
                Row(label: "gate (NIS)") { NumberField(value: $draft.filterConfig.gate) }
                Row(label: "p0_v") { NumberField(value: $draft.filterConfig.p0V) }
                Row(label: "p0_b") { NumberField(value: $draft.filterConfig.p0B) }
            }
            Row(label: "Reset to defaults") {
                QuietButton(title: "Defaults", role: Theme.inkSoft) { draft.filterConfig = .default }
            }
        }
    }

    private var flow: some View {
        VStack(alignment: .leading, spacing: 0) {
            SectionHead("Flow")
            Row(label: "Scale factor") { NumberField(value: $draft.scaleFactor) }
            Row(label: "Extra PSR floor") { NumberField(value: $draft.flowQualityThreshold) }
            Note("FLOW is OK when PSR ≥ max(floor, psr_min).")
            SwitchRow(label: "Zero-velocity updates", isOn: $draft.zuptEnabled)
        }
    }
}

private struct LiveValue: View {
    let label: String
    let value: Double
    let threshold: Double
    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Micro(label)
            Text(String(format: "%+.2f", value))
                .font(Theme.mono(18))
                .foregroundStyle(value > threshold ? Theme.live : (value < -threshold ? Theme.alert : Theme.ink))
        }
    }
}
