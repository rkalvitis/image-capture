import AVFoundation
import CoreMedia
import Foundation
import simd
import UIKit

enum CameraError: Error, CustomStringConvertible {
    case noUltraWide, configuration(String), capture(String), noRawFormat
    var description: String {
        switch self {
        case .noUltraWide: return "no builtInUltraWideCamera on this device"
        case .configuration(let s): return "configuration: \(s)"
        case .capture(let s): return "capture: \(s)"
        case .noRawFormat: return "no RAW pixel format offered on this camera"
        }
    }
}

/// Physical ultra-wide camera, locked focus/exposure/WB, ProRAW capture,
/// per-frame intrinsics from a video data output connection.
final class CameraController: NSObject {
    let session = AVCaptureSession()
    private let sessionQueue = DispatchQueue(label: "capture.session")
    private let videoQueue = DispatchQueue(label: "capture.video")
    private var device: AVCaptureDevice?
    private let photoOutput = AVCapturePhotoOutput()
    private let videoOutput = AVCaptureVideoDataOutput()
    private var rawFormat: OSType?
    private var rawFormatProRAW: OSType?
    private var rawFormatBayer: OSType?
    /// power knobs (hello: raw, quality, frame_rate) — see setPower()
    var qualityPrioritization: AVCapturePhotoOutput.QualityPrioritization = .quality
    private(set) var rawKind = "proraw"
    private var delegates: [Int64: PhotoDelegate] = [:]
    private let intrinsicsLock = NSLock()
    private var latestIntrinsics: (m: simd_float3x3, w: Int32, h: Int32)?

    var log: (String) -> Void = { print($0) }
    var settleMs: Int = 300
    private(set) var formatDescription = "-"
    private(set) var photoCalibrationAvailable = false
    private var gdcEnabled: Bool? = nil

    // MARK: setup
    func configure(completion: @escaping (Result<String, Error>) -> Void) {
        AVCaptureDevice.requestAccess(for: .video) { granted in
            guard granted else { completion(.failure(CameraError.configuration("camera permission denied"))); return }
            self.sessionQueue.async {
                do {
                    try self.setupSession()
                    self.session.startRunning()
                    completion(.success(self.formatDescription))
                } catch {
                    completion(.failure(error))
                }
            }
        }
    }

    private func setupSession() throws {
        guard let dev = AVCaptureDevice.default(.builtInUltraWideCamera, for: .video, position: .back) else {
            throw CameraError.noUltraWide
        }
        device = dev
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .photo

        let input = try AVCaptureDeviceInput(device: dev)
        guard session.canAddInput(input) else { throw CameraError.configuration("cannot add ultra-wide input") }
        session.addInput(input)

        guard session.canAddOutput(photoOutput) else { throw CameraError.configuration("cannot add photo output") }
        session.addOutput(photoOutput)
        photoOutput.maxPhotoQualityPrioritization = .quality
        // ProRAW must be enabled before the session runs (lengthy reconfiguration otherwise)
        photoOutput.isAppleProRAWEnabled = photoOutput.isAppleProRAWSupported

        // pick the RAW pixel format now: ProRAW if offered, Bayer RAW otherwise
        let proraw = photoOutput.availableRawPhotoPixelFormatTypes.first(where: { AVCapturePhotoOutput.isAppleProRAWPixelFormat($0) })
        let bayer = photoOutput.availableRawPhotoPixelFormatTypes.first(where: { AVCapturePhotoOutput.isBayerRAWPixelFormat($0) })
        rawFormatProRAW = proraw; rawFormatBayer = bayer
        rawFormat = proraw ?? bayer
        rawKind = proraw != nil ? "proraw" : "bayer"
        let dims = CMVideoFormatDescriptionGetDimensions(dev.activeFormat.formatDescription)
        formatDescription = (proraw != nil ? "ProRAW" : (bayer != nil ? "BayerRAW" : "NO-RAW")) + " \(dims.width)x\(dims.height)"

        // per-frame intrinsics come from a video connection
        videoOutput.alwaysDiscardsLateVideoFrames = true
        videoOutput.setSampleBufferDelegate(self, queue: videoQueue)
        if session.canAddOutput(videoOutput) {
            session.addOutput(videoOutput)
            if let c = videoOutput.connection(with: .video), c.isCameraIntrinsicMatrixDeliverySupported {
                c.isCameraIntrinsicMatrixDeliveryEnabled = true
            } else {
                log("intrinsic matrix delivery NOT supported on the video connection")
            }
        }
        // photo-level calibration data (only virtual devices + GDC off) — record whether it exists
        photoCalibrationAvailable = photoOutput.isCameraCalibrationDataDeliverySupported
        if dev.isGeometricDistortionCorrectionSupported { gdcEnabled = dev.isGeometricDistortionCorrectionEnabled }

        try dev.lockForConfiguration()
        if dev.isSmoothAutoFocusSupported { dev.isSmoothAutoFocusEnabled = false }
        if dev.isFocusModeSupported(.locked) { dev.setFocusModeLocked(lensPosition: 0.25) { _ in } }
        dev.unlockForConfiguration()
        log("device \(dev.localizedName), raw \(formatDescription), proraw supported \(photoOutput.isAppleProRAWSupported), calib \(photoCalibrationAvailable), gdc \(String(describing: gdcEnabled))")
    }

    // MARK: settings
    /// exposureMode / wbMode: "auto" (continuous), "locked" (meter now, then hold),
    /// "custom" (explicit duration/iso or temperature/tint).
    func applySessionSettings(lensPosition: Float, exposureMode: String, exposureDuration: Double?, iso: Double?,
                              wbMode: String, wbTemperature: Double?, wbTint: Double?) throws {
        guard let dev = device else { throw CameraError.configuration("no device") }
        try dev.lockForConfiguration()
        // exposure
        switch exposureMode {
        case "custom":
            if let d = exposureDuration, let i = iso, dev.isExposureModeSupported(.custom) {
                let fmt = dev.activeFormat
                let dur = min(max(d, fmt.minExposureDuration.seconds), fmt.maxExposureDuration.seconds)
                let isoV = Float(min(max(i, Double(fmt.minISO)), Double(fmt.maxISO)))
                dev.setExposureModeCustom(duration: CMTime(seconds: dur, preferredTimescale: 1_000_000_000), iso: isoV) { _ in }
            }
        default:   // auto and locked both start by metering
            if dev.isExposureModeSupported(.continuousAutoExposure) { dev.exposureMode = .continuousAutoExposure }
        }
        // white balance
        switch wbMode {
        case "custom":
            if let k = wbTemperature, dev.isLockingWhiteBalanceWithCustomDeviceGainsSupported {
                var g = dev.deviceWhiteBalanceGains(for: AVCaptureDevice.WhiteBalanceTemperatureAndTintValues(temperature: Float(k), tint: Float(wbTint ?? 0)))
                let maxG = dev.maxWhiteBalanceGain
                g.redGain = min(max(g.redGain, 1), maxG); g.greenGain = min(max(g.greenGain, 1), maxG); g.blueGain = min(max(g.blueGain, 1), maxG)
                dev.setWhiteBalanceModeLocked(with: g) { _ in }
            }
        default:
            if dev.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) { dev.whiteBalanceMode = .continuousAutoWhiteBalance }
        }
        if dev.hasTorch { dev.torchMode = .off }
        dev.unlockForConfiguration()
        _ = try setLensPosition(lensPosition, settleMs: 0)          // waits for the lock

        // "locked": let auto settle, then freeze what it found (consistent for the whole session)
        if exposureMode == "locked" || wbMode == "locked" {
            Thread.sleep(forTimeInterval: 1.5)
            try dev.lockForConfiguration()
            if exposureMode == "locked", dev.isExposureModeSupported(.locked) { dev.exposureMode = .locked }
            if wbMode == "locked", dev.isWhiteBalanceModeSupported(.locked) { dev.whiteBalanceMode = .locked }
            dev.unlockForConfiguration()
        }
        log("settings: exposure \(exposureMode) \(exposureDescription()), wb \(wbMode), lens \(lensPosition)")
    }

    /// Heat reduction, all optional (hello / ui): `raw` "proraw" (default, on-phone computational
    /// processing per frame) or "bayer" (sensor data only, demosaic on the Mac); `quality`
    /// "quality" | "balanced" | "speed" (ProRAW processing effort); `frame_rate` caps the session
    /// frame rate (preview + intrinsics tap; the photo is unaffected). Returns a description.
    func setPower(raw: String?, quality: String?, frameRate: Int?) throws -> String {
        guard let dev = device else { throw CameraError.configuration("no device") }
        if let r = raw {
            switch r {
            case "bayer":
                guard let f = rawFormatBayer else { throw CameraError.configuration("Bayer RAW not offered") }
                rawFormat = f; rawKind = "bayer"
            case "proraw":
                guard let f = rawFormatProRAW else { throw CameraError.configuration("ProRAW not offered") }
                rawFormat = f; rawKind = "proraw"
            default: throw CameraError.configuration("raw must be proraw | bayer")
            }
            let dims = CMVideoFormatDescriptionGetDimensions(dev.activeFormat.formatDescription)
            formatDescription = (rawKind == "proraw" ? "ProRAW" : "BayerRAW") + " \(dims.width)x\(dims.height)"
        }
        if let q = quality {
            switch q {
            case "quality": qualityPrioritization = .quality
            case "balanced": qualityPrioritization = .balanced
            case "speed": qualityPrioritization = .speed
            default: throw CameraError.configuration("quality must be quality | balanced | speed")
            }
        }
        if let fps = frameRate {
            try dev.lockForConfiguration()
            let ranges = dev.activeFormat.videoSupportedFrameRateRanges
            let minFps = ranges.map { $0.minFrameRate }.min() ?? 1
            let maxFps = ranges.map { $0.maxFrameRate }.max() ?? 30
            let f = min(max(Double(fps), minFps), maxFps)
            dev.activeVideoMinFrameDuration = CMTime(value: 1, timescale: CMTimeScale(f))
            dev.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: CMTimeScale(f))
            dev.unlockForConfiguration()
        }
        let fpsNow = dev.activeVideoMinFrameDuration.seconds > 0 ? 1.0 / dev.activeVideoMinFrameDuration.seconds : 0
        let desc = "raw \(rawKind), quality \(qualityPrioritization.rawValue), \(Int(fpsNow.rounded())) fps"
        log("power: " + desc)
        return desc
    }

    /// Lock the lens at `position`, wait for the completion + settle, return the applied value.
    func setLensPosition(_ position: Float, settleMs: Int) throws -> Float {
        guard let dev = device else { throw CameraError.configuration("no device") }
        let sem = DispatchSemaphore(value: 0)
        try dev.lockForConfiguration()
        dev.setFocusModeLocked(lensPosition: min(max(position, 0), 1)) { _ in sem.signal() }
        dev.unlockForConfiguration()
        _ = sem.wait(timeout: .now() + 3)
        Thread.sleep(forTimeInterval: Double(settleMs) / 1000.0)
        return dev.lensPosition
    }

    /// Autofocus on the frame centre (the robot puts the beetle there): continuous AF for
    /// a bounded time (one-shot `.autoFocus` does not scan at macro distances), wait until
    /// the lens stops moving, then re-lock at the position found and return it.
    func autoFocus(settleMs: Int, timeoutS: Double = 4.0) throws -> Float {
        guard let dev = device else { throw CameraError.configuration("no device") }
        let mode: AVCaptureDevice.FocusMode = dev.isFocusModeSupported(.continuousAutoFocus) ? .continuousAutoFocus : .autoFocus
        guard dev.isFocusModeSupported(mode) else { throw CameraError.configuration("autofocus not supported") }
        let start = dev.lensPosition
        try dev.lockForConfiguration()
        if dev.isFocusPointOfInterestSupported { dev.focusPointOfInterest = CGPoint(x: 0.5, y: 0.5) }
        dev.focusMode = mode
        dev.unlockForConfiguration()
        let t0 = Date()
        var last = dev.lensPosition, lastChange = Date(), moved = false
        while Date().timeIntervalSince(t0) < timeoutS {
            Thread.sleep(forTimeInterval: 0.02)
            let lp = dev.lensPosition
            if abs(lp - last) > 0.0005 { last = lp; lastChange = Date(); moved = true }
            // settled: not adjusting and the lens has not moved for 0.5 s (after at least 0.6 s of AF)
            if !dev.isAdjustingFocus && Date().timeIntervalSince(lastChange) > 0.5 && Date().timeIntervalSince(t0) > 0.6 { break }
        }
        let found = dev.lensPosition
        log(String(format: "autofocus (%@): %.4f -> %.4f after %.2f s%@", mode == .continuousAutoFocus ? "continuous" : "one-shot",
                   start, found, Date().timeIntervalSince(t0), moved ? "" : " (lens did not move)"))
        return try setLensPosition(found, settleMs: settleMs)
    }

    func currentLensPosition() -> Float { device?.lensPosition ?? -1 }
    func currentZoomFactor() -> Double { Double(device?.videoZoomFactor ?? -1) }

    func exposureDescription() -> String {
        guard let d = device else { return "-" }
        return String(format: "1/%.0f s ISO %.0f", 1.0 / max(d.exposureDuration.seconds, 1e-6), d.iso)
    }

    func latestIntrinsicsDict() -> [String: Any]? {
        intrinsicsLock.lock(); defer { intrinsicsLock.unlock() }
        guard let i = latestIntrinsics else { return nil }
        return ["fx": i.m.columns.0.x, "fy": i.m.columns.1.y, "cx": i.m.columns.2.x, "cy": i.m.columns.2.y,
                "w": i.w, "h": i.h]
    }

    // MARK: capture
    /// Blocking: one frame per lens position, files written via `store`. Returns frame dicts (PROTOCOL.md `done.frames`).
    func captureStack(session: String, stop: Int, pose: Int, step: Int, lensPositions: [Float], settleMs: Int,
                      store: FrameStore, thermal: String,
                      onFrame: @escaping ([String: Any]) -> Void) -> Result<[[String: Any]], Error> {
        guard let dev = device, let raw = rawFormat else { return .failure(CameraError.noRawFormat) }
        var frames: [[String: Any]] = []
        for (k, lp) in lensPositions.enumerated() {
            do {
                // thermal throttle: slow down when hot, stop when critical (max 5 min wait)
                var waited = 0.0
                while ProcessInfo.processInfo.thermalState == .critical && waited < 300 {
                    if waited == 0 { log("thermal CRITICAL — pausing capture") }
                    Thread.sleep(forTimeInterval: 2); waited += 2
                }
                if ProcessInfo.processInfo.thermalState == .serious {
                    if k == 0 { log("thermal serious — 3 s pause between frames") }
                    Thread.sleep(forTimeInterval: 3)
                }
                let applied = try setLensPosition(lp, settleMs: settleMs)
                let settings = AVCapturePhotoSettings(rawPixelFormatType: raw)
                settings.photoQualityPrioritization = qualityPrioritization
                settings.flashMode = .off
                if photoCalibrationAvailable && photoOutput.isCameraCalibrationDataDeliverySupported {
                    settings.isCameraCalibrationDataDeliveryEnabled = true
                }
                let videoIntrinsics = latestIntrinsicsDict()
                let delegate = PhotoDelegate()
                delegates[settings.uniqueID] = delegate
                let sem = DispatchSemaphore(value: 0)
                delegate.onDone = { sem.signal() }
                sessionQueue.async { self.photoOutput.capturePhoto(with: settings, delegate: delegate) }
                if sem.wait(timeout: .now() + 20) == .timedOut {
                    delegates[settings.uniqueID] = nil
                    return .failure(CameraError.capture("timeout waiting for the photo"))
                }
                delegates[settings.uniqueID] = nil
                if let e = delegate.error { return .failure(CameraError.capture("\(e)")) }
                guard let photo = delegate.photo, let data = photo.fileDataRepresentation() else {
                    return .failure(CameraError.capture("no DNG data"))
                }
                // capture time: photo.timestamp is on the host clock -> map to wall time
                let hostNow = CMClockGetTime(CMClockGetHostTimeClock()).seconds
                let wallNow = Date().timeIntervalSince1970
                let captureTime = wallNow - (hostNow - photo.timestamp.seconds)
                let dims = photo.pixelBuffer.map { (CVPixelBufferGetWidth($0), CVPixelBufferGetHeight($0)) }
                let photoW = dims?.0 ?? 4032, photoH = dims?.1 ?? 3024
                var intrinsics: [String: Any]? = nil
                if let vi = videoIntrinsics, let vw = vi["w"] as? Int32, let vh = vi["h"] as? Int32, vw > 0 {
                    let sx = Double(photoW) / Double(vw), sy = Double(photoH) / Double(vh)
                    intrinsics = ["fx": Double(vi["fx"] as! Float) * sx, "fy": Double(vi["fy"] as! Float) * sy,
                                  "cx": Double(vi["cx"] as! Float) * sx, "cy": Double(vi["cy"] as! Float) * sy,
                                  "w": photoW, "h": photoH]
                }
                let exif = (photo.metadata["{Exif}"] as? [String: Any]) ?? [:]
                let name = String(format: "IMG_%@_%04d_%02d", session, stop, k)
                let sidecar: [String: Any] = [
                    "file": name + ".dng", "session": session, "stop": stop, "pose": pose, "step": step,
                    "focus_index": k, "lens_position": applied, "lens_position_requested": lp,
                    "capture_time": captureTime,
                    "exposure": ["duration_s": dev.exposureDuration.seconds, "iso": dev.iso],
                    "white_balance_gains": ["r": dev.deviceWhiteBalanceGains.redGain, "g": dev.deviceWhiteBalanceGains.greenGain, "b": dev.deviceWhiteBalanceGains.blueGain],
                    "intrinsics": intrinsics as Any,
                    "intrinsics_video": videoIntrinsics as Any,
                    "calibration": CameraController.calibrationDict(photo.cameraCalibrationData) as Any,
                    "gdc_enabled": gdcEnabled as Any,
                    "orientation": photo.metadata[kCGImagePropertyOrientation as String] as Any,
                    "raw_format": formatDescription, "is_raw": photo.isRawPhoto,
                    "zoom_factor": Double(dev.videoZoomFactor),
                    "exif": ["ExposureTime": exif["ExposureTime"] as Any, "ISOSpeedRatings": exif["ISOSpeedRatings"] as Any,
                             "DateTimeOriginal": exif["DateTimeOriginal"] as Any, "SubsecTimeOriginal": exif["SubsecTimeOriginal"] as Any],
                    "device": UIDevice.current.model + " " + UIDevice.current.systemVersion, "lens": "ultra-wide",
                    "thermal": thermal,
                ]
                try store.write(session: session, name: name, dng: data, sidecar: sidecar)
                let frame: [String: Any] = ["k": k, "file": name + ".dng", "capture_time": captureTime,
                                            "lens_position": applied, "intrinsics": intrinsics as Any,
                                            "exposure": sidecar["exposure"]!, "thermal": thermal]
                onFrame(frame)
                frames.append(frame)
            } catch {
                return .failure(error)
            }
        }
        return .success(frames)
    }

    static func calibrationDict(_ c: AVCameraCalibrationData?) -> [String: Any]? {
        guard let c else { return nil }
        let m = c.intrinsicMatrix
        let e = c.extrinsicMatrix
        func lut(_ d: Data?) -> [Float]? {
            guard let d else { return nil }
            return d.withUnsafeBytes { Array($0.bindMemory(to: Float.self)) }
        }
        return [
            "intrinsic_matrix": [[m.columns.0.x, m.columns.1.x, m.columns.2.x],
                                 [m.columns.0.y, m.columns.1.y, m.columns.2.y],
                                 [m.columns.0.z, m.columns.1.z, m.columns.2.z]],
            "intrinsic_reference_dimensions": [c.intrinsicMatrixReferenceDimensions.width, c.intrinsicMatrixReferenceDimensions.height],
            "extrinsic_matrix": [[e.columns.0.x, e.columns.1.x, e.columns.2.x, e.columns.3.x],
                                 [e.columns.0.y, e.columns.1.y, e.columns.2.y, e.columns.3.y],
                                 [e.columns.0.z, e.columns.1.z, e.columns.2.z, e.columns.3.z]],
            "pixel_size_mm": c.pixelSize,
            "lens_distortion_center": [c.lensDistortionCenter.x, c.lensDistortionCenter.y],
            "lens_distortion_lut": lut(c.lensDistortionLookupTable) as Any,
            "inverse_lens_distortion_lut": lut(c.inverseLensDistortionLookupTable) as Any,
        ]
    }
}

// MARK: - intrinsics tap
extension CameraController: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let att = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil) as? Data,
              att.count >= MemoryLayout<simd_float3x3>.size,
              let fd = CMSampleBufferGetFormatDescription(sampleBuffer) else { return }
        let m = att.withUnsafeBytes { $0.load(as: simd_float3x3.self) }
        let dims = CMVideoFormatDescriptionGetDimensions(fd)
        intrinsicsLock.lock(); latestIntrinsics = (m, dims.width, dims.height); intrinsicsLock.unlock()
    }
}

// MARK: - photo delegate
final class PhotoDelegate: NSObject, AVCapturePhotoCaptureDelegate {
    var photo: AVCapturePhoto?
    var error: Error?
    var onDone: (() -> Void)?

    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        if let error { self.error = error }
        if photo.isRawPhoto { self.photo = photo }
    }

    func photoOutput(_ output: AVCapturePhotoOutput, didFinishCaptureFor resolvedSettings: AVCaptureResolvedPhotoSettings, error: Error?) {
        if let error, self.error == nil { self.error = error }
        onDone?()
    }
}
