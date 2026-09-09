import Foundation
import UIKit
import Combine

/// UI state + glue between the control server and the camera.
final class AppState: ObservableObject {
    @Published var serverStatus = "starting"
    @Published var sessionId: String?
    @Published var lastCommand = "-"
    @Published var lensPosition: Float?
    @Published var exposureText = "-"
    @Published var formatText = "-"
    @Published var captureCount = 0
    @Published var thermal = "nominal"
    @Published var freeText = "-"
    @Published var cameraReady = false
    @Published var log: [String] = []

    let camera = CameraController()
    let store = FrameStore()
    private var server: ControlServer?
    private var started = false
    private var benchStop = 0

    static let port: UInt16 = 5601

    func start() {
        guard !started else { return }
        started = true
        UIApplication.shared.isIdleTimerDisabled = true
        UIDevice.current.isBatteryMonitoringEnabled = true
        camera.log = { [weak self] m in self?.addLog(m) }
        camera.configure { [weak self] result in
            DispatchQueue.main.async {
                guard let self else { return }
                switch result {
                case .success(let desc):
                    self.cameraReady = true
                    self.formatText = desc
                    self.addLog("camera ready: \(desc)")
                case .failure(let err):
                    self.formatText = "ERROR"
                    self.addLog("camera error: \(err)")
                }
            }
        }
        server = ControlServer(port: AppState.port, handler: self)
        server?.onStatus = { [weak self] s in DispatchQueue.main.async { self?.serverStatus = s } }
        server?.start()
        NotificationCenter.default.addObserver(forName: ProcessInfo.thermalStateDidChangeNotification, object: nil, queue: .main) { [weak self] _ in
            self?.thermal = AppState.thermalName()
        }
        thermal = AppState.thermalName()
        refreshFree()
    }

    static func thermalName() -> String {
        switch ProcessInfo.processInfo.thermalState {
        case .nominal: return "nominal"
        case .fair: return "fair"
        case .serious: return "serious"
        case .critical: return "critical"
        @unknown default: return "unknown"
        }
    }

    func refreshFree() {
        let free = store.freeBytes()
        DispatchQueue.main.async { self.freeText = String(format: "%.1f GB", Double(free) / 1e9) }
    }

    func addLog(_ m: String) {
        DispatchQueue.main.async {
            let ts = DateFormatter.localizedString(from: Date(), dateStyle: .none, timeStyle: .medium)
            self.log.append("\(ts) \(m)")
            if self.log.count > 200 { self.log.removeFirst(self.log.count - 200) }
        }
    }

    func setLens(_ v: Float) {
        DispatchQueue.global(qos: .userInitiated).async {
            if let applied = try? self.camera.setLensPosition(v, settleMs: 0) {
                DispatchQueue.main.async { self.lensPosition = applied }
            }
        }
    }

    // -- bench button ----------------------------------------------------
    func benchShoot(start: Double, end: Double, frames: Int) {
        let session = sessionId ?? "bench"
        benchStop += 1
        let stop = benchStop
        let n = max(1, frames)
        let positions: [Float] = (0..<n).map { i in Float(n == 1 ? start : start + (end - start) * Double(i) / Double(n - 1)) }
        addLog("bench stack: \(n) frame(s) \(start)→\(end)")
        DispatchQueue.global(qos: .userInitiated).async {
            let res = self.camera.captureStack(session: session, stop: stop, pose: 0, step: 0,
                                               lensPositions: positions,
                                               settleMs: self.camera.settleMs, store: self.store, thermal: AppState.thermalName(),
                                               onFrame: { f in DispatchQueue.main.async { self.captureCount += 1; self.lensPosition = f["lens_position"] as? Float } })
            switch res {
            case .success(let frames): self.addLog("bench: \(frames.count) frame(s) -> \(frames.first?["file"] ?? "")")
            case .failure(let e): self.addLog("bench failed: \(e)")
            }
            self.refreshFree()
        }
    }
}

// MARK: - Command handling (docs/PROTOCOL.md)

extension AppState: ControlHandler {
    /// Called on the server's queue. `send` streams intermediate messages (frame).
    func handle(_ msg: [String: Any], send: @escaping ([String: Any]) -> Void) -> [String: Any] {
        let type = msg["type"] as? String ?? "?"
        DispatchQueue.main.async { self.lastCommand = type }
        let now = Date().timeIntervalSince1970
        switch type {
        case "ping":
            return ["type": "pong", "t": now]

        case "hello":
            guard let session = msg["session"] as? String else {
                return ["type": "error", "reason": "hello without session", "t": now]
            }
            let lens = (msg["lens_positions"] as? [Double])?.first ?? 0.5
            let settle = msg["settle_ms"] as? Int ?? 300
            let exposure = msg["exposure"] as? [String: Any] ?? [:]
            let wb = msg["wb"] as? [String: Any] ?? [:]
            let expMode = (exposure["mode"] as? String) ?? ((exposure["duration_s"] != nil && exposure["iso"] != nil) ? "custom" : "auto")
            let wbModeStr = (wb["mode"] as? String) ?? (wb["temperature_k"] != nil ? "custom" : "auto")
            do {
                try camera.applySessionSettings(lensPosition: Float(lens),
                                                exposureMode: expMode,
                                                exposureDuration: exposure["duration_s"] as? Double,
                                                iso: exposure["iso"] as? Double,
                                                wbMode: wbModeStr,
                                                wbTemperature: wb["temperature_k"] as? Double,
                                                wbTint: wb["tint"] as? Double)
            } catch {
                return ["type": "error", "reason": "settings: \(error)", "t": now]
            }
            store.session = session
            camera.settleMs = settle
            DispatchQueue.main.async {
                self.sessionId = session
                self.lensPosition = self.camera.currentLensPosition()
                self.exposureText = self.camera.exposureDescription()
            }
            addLog("hello: session \(session), lens \(lens)")
            return ["type": "ready", "t": now,
                    "device": UIDevice.current.model + " " + UIDevice.current.systemVersion,
                    "lens": "ultra-wide",
                    "format": camera.formatDescription,
                    "free_bytes": store.freeBytes(),
                    "calib_available": camera.photoCalibrationAvailable,
                    "intrinsics_video": camera.latestIntrinsicsDict() as Any]

        case "focus":
            guard let lp = msg["lens_position"] as? Double else {
                return ["type": "error", "reason": "focus without lens_position", "t": now]
            }
            do {
                let applied = try camera.setLensPosition(Float(lp), settleMs: camera.settleMs)
                DispatchQueue.main.async { self.lensPosition = applied }
                return ["type": "focus", "t": now, "lens_position": applied]
            } catch {
                return ["type": "error", "reason": "focus: \(error)", "t": now]
            }

        case "stack":
            guard let session = store.session else {
                return ["type": "error", "reason": "stack before hello", "t": now]
            }
            let id = msg["id"] as? Int ?? 0
            let pose = msg["pose"] as? Int ?? 0
            let step = msg["step"] as? Int ?? 0
            let lens = (msg["lens_positions"] as? [Double])?.map { Float($0) } ?? [camera.currentLensPosition()]
            if store.freeBytes() < 2_000_000_000 {
                return ["type": "error", "id": id, "reason": "less than 2 GB free", "t": now]
            }
            let res = camera.captureStack(session: session, stop: id, pose: pose, step: step,
                                          lensPositions: lens, settleMs: camera.settleMs, store: store,
                                          thermal: AppState.thermalName(),
                                          onFrame: { frame in
                                              var f = frame; f["type"] = "frame"; f["id"] = id; send(f)
                                              DispatchQueue.main.async { self.captureCount += 1 }
                                          })
            DispatchQueue.main.async { self.lensPosition = self.camera.currentLensPosition() }
            refreshFree()
            switch res {
            case .success(let frames):
                return ["type": "done", "id": id, "t": Date().timeIntervalSince1970, "frames": frames]
            case .failure(let err):
                return ["type": "error", "id": id, "reason": "\(err)", "t": Date().timeIntervalSince1970]
            }

        case "status":
            return ["type": "status", "t": now, "thermal": AppState.thermalName(),
                    "free_bytes": store.freeBytes(),
                    "battery": Double(UIDevice.current.batteryLevel),
                    "charging": UIDevice.current.batteryState == .charging || UIDevice.current.batteryState == .full,
                    "lens_position": camera.currentLensPosition(),
                    "zoom_factor": camera.currentZoomFactor(),
                    "session": store.session as Any]

        case "ui":
            // layout control from the Mac: {"horizontal": bool, "clips": bool, "fill": bool}
            let d = UserDefaults.standard
            if let v = msg["horizontal"] as? Bool { d.set(v, forKey: "uiHorizontal") }
            if let v = msg["clips"] as? Bool { d.set(v, forKey: "showClips") }
            if let v = msg["fill"] as? Bool { d.set(v, forKey: "previewFill") }
            return ["type": "ui", "t": now, "horizontal": d.bool(forKey: "uiHorizontal"),
                    "clips": d.bool(forKey: "showClips"), "fill": d.bool(forKey: "previewFill")]

        case "bye":
            addLog("bye")
            return ["type": "status", "t": now, "thermal": AppState.thermalName()]

        default:
            return ["type": "error", "reason": "unknown command \(type)", "t": now]
        }
    }
}
