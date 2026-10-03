import SwiftUI
import UIKit

@main
struct GroundSpeedApp: App {
    @StateObject private var model = AppModel()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            MainView()
                .environmentObject(model)
                .statusBarHidden(false)
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = true   // keep the screen awake
                    model.startIfNeeded()
                }
                .onChange(of: scenePhase) { _, phase in
                    model.handleScenePhase(phase)
                }
        }
    }
}
