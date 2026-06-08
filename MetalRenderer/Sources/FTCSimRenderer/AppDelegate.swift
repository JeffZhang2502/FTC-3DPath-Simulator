import AppKit
import SwiftUI

final class AppDelegate: NSObject, NSApplicationDelegate {

    private var window: NSWindow!

    func applicationDidFinishLaunching(_ notification: Notification) {
        print("[AppDelegate] Creating window...")
        window = NSWindow(
            contentRect: NSRect(x: 0, y: 0, width: 1100, height: 760),
            styleMask: [.titled, .closable, .miniaturizable, .resizable],
            backing: .buffered,
            defer: false
        )
        window.title = "FTC Auto Simulator — DECODE 2025-2026 [Metal]"
        window.minSize = NSSize(width: 800, height: 600)
        window.center()
        window.contentView = NSHostingView(rootView: ContentView())
        print("[AppDelegate] Showing window...")
        window.makeKeyAndOrderFront(nil)
        print("[AppDelegate] Window visible.")
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        true
    }
}
