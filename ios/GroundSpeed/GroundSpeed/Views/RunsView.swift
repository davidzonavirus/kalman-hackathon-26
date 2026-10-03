import PhoneRuntime
import SwiftUI
import UIKit

/// Run log: each recorded run, exported as a zip through the share sheet (AirDrop, Files…).
/// Runs are also visible in the Files app under On My iPhone › GroundSpeed › runs.
struct RunsPane: View {
    @EnvironmentObject var model: AppModel
    @State private var shareItem: ShareItem?
    @State private var exporting: String?
    @State private var pendingDelete: RunSummary?

    struct ShareItem: Identifiable {
        let id = UUID()
        let url: URL
    }

    var body: some View {
        Group {
            if model.runs.isEmpty {
                VStack {
                    Spacer()
                    Note("No runs yet. Press Record on the main screen.")
                    Spacer()
                }
                .frame(maxWidth: .infinity)
            } else {
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 0) {
                        ForEach(model.runs) { run in
                            RunRow(run: run,
                                   isActive: model.isRecording && run.runId == model.runId,
                                   exporting: exporting == run.runId,
                                   onShare: { share(run) },
                                   onDelete: { pendingDelete = run })
                            Hairline()
                        }
                    }
                    .padding(.horizontal, 24)
                    .padding(.bottom, 40)
                }
            }
        }
        .onAppear { model.refreshRuns() }
        .sheet(item: $shareItem) { item in
            ShareSheet(items: [item.url])
        }
        .confirmationDialog("Delete this run?",
                            isPresented: Binding(get: { pendingDelete != nil },
                                                 set: { if !$0 { pendingDelete = nil } }),
                            titleVisibility: .visible) {
            Button("Delete", role: .destructive) {
                if let r = pendingDelete { model.deleteRun(r) }
                pendingDelete = nil
            }
            Button("Cancel", role: .cancel) { pendingDelete = nil }
        } message: {
            Text(pendingDelete?.runId ?? "")
        }
    }

    private func share(_ run: RunSummary) {
        exporting = run.runId
        Task { @MainActor in
            let url = await model.exportRun(run)
            exporting = nil
            if let url {
                shareItem = ShareItem(url: url)
            } else {
                model.errorMessage = "Export failed for \(run.runId)"
            }
        }
    }
}

private struct RunRow: View {
    let run: RunSummary
    let isActive: Bool
    let exporting: Bool
    let onShare: () -> Void
    let onDelete: () -> Void

    var body: some View {
        HStack(alignment: .center, spacing: 12) {
            VStack(alignment: .leading, spacing: 4) {
                Text(run.runId)
                    .font(Theme.mono(13))
                    .foregroundStyle(Theme.ink)
                    .lineLimit(1)
                    .minimumScaleFactor(0.8)
                Micro(isActive ? "recording" : ByteCountFormatter.string(fromByteCount: run.sizeBytes, countStyle: .file),
                      color: isActive ? Theme.alert : Theme.inkSoft)
            }
            Spacer()
            if exporting {
                ProgressView().tint(Theme.ink)
            } else if !isActive {
                Button(action: onShare) {
                    Image(systemName: "square.and.arrow.up")
                        .font(.system(size: 16))
                        .foregroundStyle(Theme.ink)
                        .frame(width: 44, height: 44)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
                .accessibilityLabel("Export \(run.runId)")
            }
        }
        .padding(.vertical, 10)
        .contentShape(Rectangle())
        .contextMenu {
            if !isActive {
                Button(role: .destructive, action: onDelete) {
                    Label("Delete run", systemImage: "trash")
                }
            }
        }
    }
}

/// UIActivityViewController wrapper (AirDrop, Files, Mail…).
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
