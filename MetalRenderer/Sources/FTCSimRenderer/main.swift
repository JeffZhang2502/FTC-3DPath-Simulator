import AppKit

print("[Main] Starting FTC Sim Renderer...")
let app = NSApplication.shared
let delegate = AppDelegate()
app.delegate = delegate
app.setActivationPolicy(.regular)
print("[Main] Activating app...")
app.activate(ignoringOtherApps: true)
print("[Main] Entering run loop...")
app.run()
