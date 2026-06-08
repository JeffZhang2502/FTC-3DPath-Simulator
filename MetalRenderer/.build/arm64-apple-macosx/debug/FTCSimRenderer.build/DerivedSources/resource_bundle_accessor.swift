import Foundation

extension Foundation.Bundle {
    static nonisolated let module: Bundle = {
        let mainPath = Bundle.main.bundleURL.appendingPathComponent("FTCSimRenderer_FTCSimRenderer.bundle").path
        let buildPath = "/Users/h0pe_wsv/Desktop/FTC-3DPath-Simulator-main/MetalRenderer/.build/arm64-apple-macosx/debug/FTCSimRenderer_FTCSimRenderer.bundle"

        let preferredBundle = Bundle(path: mainPath)

        guard let bundle = preferredBundle ?? Bundle(path: buildPath) else {
            // Users can write a function called fatalError themselves, we should be resilient against that.
            Swift.fatalError("could not load resource bundle: from \(mainPath) or \(buildPath)")
        }

        return bundle
    }()
}