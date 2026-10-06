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
    @Published var showPreview = true          // ui {"preview": false} / hello {"preview": false}: no live preview (less GPU + heat)
    @Published var lowFps = false              // frame rate capped at 10 fps (preview + intrinsics tap)
    @Published var dimmed = false              // screen brightness 0.02
    @Published var blackout = false            // hello/ui {"blackout": true}: the whole screen is plain black (no panel, no
                                               // preview, brightness 0) for the dataset session; long-press 2 s or `bye` ends it
    private var brightnessBeforeDim: CGFloat = 0.5
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

    // MARK: - heat buttons (panel) — the same knobs the Mac sets via hello/ui
    func toggleFps() {
        let target = lowFps ? 30 : 10
        DispatchQueue.global(qos: .userInitiated).async {
            do {
                _ = try self.camera.setPower(raw: nil, quality: nil, frameRate: target)
                DispatchQueue.main.async { self.lowFps = (target <= 10) }
                self.addLog("frame rate \(target) fps")
            } catch { self.addLog("fps: \(error)") }
        }
    }

    func togglePreview() {
        showPreview.toggle()
        addLog(showPreview ? "preview on" : "preview off")
    }

    func toggleDim() {
        if dimmed {
            UIScreen.main.brightness = brightnessBeforeDim
        } else {
            brightnessBeforeDim = max(UIScreen.main.brightness, 0.3)
            UIScreen.main.brightness = 0.02
        }
        dimmed.toggle()
        addLog(dimmed ? "screen dimmed" : "screen brightness restored")
    }

    var ecoOn: Bool { lowFps && !showPreview && dimmed }

    /// Eco = 10 fps + preview off + dim (all three), or all back.
    func setEco(_ on: Bool) {
        if (lowFps != on) { toggleFps() }
        if (showPreview == on) { togglePreview() }
        if (dimmed != on) { toggleDim() }
    }

    /// Screen brightness (0..1), live preview on/off, blackout — all on the main thread.
    private func applyScreen(brightness: Double?, preview: Bool?, blackout: Bool? = nil) {
        if brightness == nil && preview == nil && blackout == nil { return }
        let sem = DispatchSemaphore(value: 0)
        DispatchQueue.main.async {
            if let b = brightness {
                if !self.dimmed { self.brightnessBeforeDim = max(UIScreen.main.brightness, 0.3) }
                UIScreen.main.brightness = CGFloat(min(max(b, 0), 1))
                self.dimmed = b <= 0.1
            }
            if let p = preview { self.showPreview = p }
            if let k = blackout { self.setBlackout(k) }
            sem.signal()
        }
        _ = sem.wait(timeout: .now() + 2)
        addLog("screen: brightness \(brightness.map { String(format: "%.2f", $0) } ?? "-"), preview \(preview.map(String.init) ?? "-")"
               + (blackout.map { ", blackout \($0)" } ?? ""))
    }

    /// Blackout (main thread): nothing drawn, preview off, brightness 0 — the OLED shows pure black.
    /// The capture server and the camera keep running; only the UI stops.
    func setBlackout(_ on: Bool) {
        if on {
            if !dimmed { brightnessBeforeDim = max(UIScreen.main.brightness, 0.3) }
            showPreview = false
            UIScreen.main.brightness = 0
            dimmed = true
        } else {
            UIScreen.main.brightness = brightnessBeforeDim
            dimmed = false
        }
        blackout = on
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
            // optional heat reduction: raw proraw|bayer, quality quality|balanced|speed, frame_rate, screen_brightness, preview
            var power = ""
            if msg["raw"] != nil || msg["quality"] != nil || msg["frame_rate"] != nil {
                do {
                    power = try camera.setPower(raw: msg["raw"] as? String, quality: msg["quality"] as? String,
                                                frameRate: msg["frame_rate"] as? Int)
                    if let f = msg["frame_rate"] as? Int { DispatchQueue.main.async { self.lowFps = f <= 10 } }
                } catch {
                    return ["type": "error", "reason": "power: \(error)", "t": now]
                }
            }
            applyScreen(brightness: msg["screen_brightness"] as? Double, preview: msg["preview"] as? Bool,
                        blackout: msg["blackout"] as? Bool)
            DispatchQueue.main.async {
                self.sessionId = session
                self.lensPosition = self.camera.currentLensPosition()
                self.exposureText = self.camera.exposureDescription()
                self.formatText = self.camera.formatDescription
            }
            if !power.isEmpty { addLog("power: " + power) }
            addLog("hello: session \(session), lens \(lens)")
            return ["type": "ready", "t": now,
                    "device": UIDevice.current.model + " " + UIDevice.current.systemVersion,
                    "lens": "ultra-wide",
                    "format": camera.formatDescription,
                    "raw": camera.rawKind, "quality": camera.qualityPrioritization.rawValue,
                    "preview": showPreview,
                    "free_bytes": store.freeBytes(),
                    "calib_available": camera.photoCalibrationAvailable,
                    "intrinsics_video": camera.latestIntrinsicsDict() as Any]

        case "focus":
            // lens_position: 0..1 = lock there; "auto" = one-shot AF on the centre, then lock at the result
            let auto = (msg["lens_position"] as? String) == "auto" || (msg["auto"] as? Bool) == true
            guard auto || msg["lens_position"] is Double else {
                return ["type": "error", "reason": "focus without lens_position", "t": now]
            }
            do {
                let applied = auto ? try camera.autoFocus(settleMs: camera.settleMs)
                                   : try camera.setLensPosition(Float(msg["lens_position"] as! Double), settleMs: camera.settleMs)
                DispatchQueue.main.async { self.lensPosition = applied }
                addLog(auto ? String(format: "autofocus -> %.4f", applied) : String(format: "focus %.4f", applied))
                return ["type": "focus", "t": now, "lens_position": applied, "auto": auto]
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
            // layout control from the Mac: {"horizontal": bool, "clips": bool, "fill": bool,
            //   "preview": bool, "screen_brightness": 0..1, "raw": proraw|bayer, "quality": ..., "frame_rate": n}
            let d = UserDefaults.standard
            if let v = msg["horizontal"] as? Bool { d.set(v, forKey: "uiHorizontal") }
            if let v = msg["clips"] as? Bool { d.set(v, forKey: "showClips") }
            if let v = msg["fill"] as? Bool { d.set(v, forKey: "previewFill") }
            applyScreen(brightness: msg["screen_brightness"] as? Double, preview: msg["preview"] as? Bool,
                        blackout: msg["blackout"] as? Bool)
            var power = ""
            if msg["raw"] != nil || msg["quality"] != nil || msg["frame_rate"] != nil {
                do {
                    power = try camera.setPower(raw: msg["raw"] as? String, quality: msg["quality"] as? String,
                                                frameRate: msg["frame_rate"] as? Int)
                    let f = msg["frame_rate"] as? Int
                    DispatchQueue.main.async {
                        self.formatText = self.camera.formatDescription
                        if let f { self.lowFps = f <= 10 }
                    }
                } catch {
                    return ["type": "error", "reason": "power: \(error)", "t": now]
                }
            }
            return ["type": "ui", "t": now, "horizontal": d.bool(forKey: "uiHorizontal"),
                    "clips": d.bool(forKey: "showClips"), "fill": d.bool(forKey: "previewFill"),
                    "preview": showPreview, "screen_brightness": Double(UIScreen.main.brightness),
                    "power": power, "raw": camera.rawKind, "quality": camera.qualityPrioritization.rawValue,
                    "thermal": AppState.thermalName()]

        case "bye":
            addLog("bye")
            applyScreen(brightness: nil, preview: nil, blackout: false)      // the screen comes back after the session
            return ["type": "status", "t": now, "thermal": AppState.thermalName()]

        default:
            return ["type": "error", "reason": "unknown command \(type)", "t": now]
        }
    }
}
