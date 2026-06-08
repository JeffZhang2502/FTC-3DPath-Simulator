import SwiftUI
import MetalKit

/// NSViewRepresentable wrapping MTKView for the Metal 3D viewport.
/// Owns the Renderer and handles camera mouse controls.
struct MetalView: NSViewRepresentable {

    @ObservedObject var client: SimClient
    @Binding var brightness: Float

    func makeNSView(context: Context) -> MTKView {
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal is not supported on this device")
        }
        let view = MTKView(frame: .zero, device: device)
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.sampleCount = 4
        view.clearColor = MTLClearColor(red: 0.55, green: 0.60, blue: 0.68, alpha: 1)

        let renderer = Renderer(device: device, view: view)
        context.coordinator.renderer = renderer

        // Mouse drag → camera orbit
        let pan = NSPanGestureRecognizer(target: context.coordinator,
                                         action: #selector(Coordinator.pan(_:)))
        view.addGestureRecognizer(pan)

        return view
    }

    func updateNSView(_ nsView: MTKView, context: Context) {
        context.coordinator.renderer?.fieldElements = client.fieldElements
        context.coordinator.renderer?.frame = client.frame
        context.coordinator.renderer?.brightness = brightness
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    class Coordinator: NSObject {
        var renderer: Renderer?
        private var lastX: CGFloat = 0
        private var lastY: CGFloat = 0

        override init() {
            super.init()
            // Scroll wheel monitor
            NSEvent.addLocalMonitorForEvents(matching: .scrollWheel) { [weak self] event in
                self?.handleScroll(event)
                return event
            }
        }

        @objc func pan(_ gesture: NSPanGestureRecognizer) {
            let loc = gesture.translation(in: gesture.view)
            switch gesture.state {
            case .began:
                lastX = 0; lastY = 0
            case .changed:
                let dx = loc.x - lastX
                let dy = loc.y - lastY
                lastX = loc.x; lastY = loc.y
                renderer?.camera.azimuth -= Float(dx) * 0.3
                renderer?.camera.elevation -= Float(dy) * 0.3   // invert: drag down = orbit up
                renderer?.camera.elevation = max(5, min(85, renderer!.camera.elevation))
            default: break
            }
        }

        func handleScroll(_ event: NSEvent) {
            guard let r = renderer else { return }
            r.camera.distance -= Float(event.deltaY) * 0.5
            r.camera.distance = max(60, min(300, r.camera.distance))
        }
    }
}
