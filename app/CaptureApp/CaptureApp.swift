import SwiftUI
import AVFoundation

@main
struct CaptureApp: App {
    @StateObject private var state = AppState()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(state)
                .onAppear { state.start() }
                .statusBarHidden(true)
        }
    }
}

/// Layout rule (from the cradle CAD, phone_mount_conf3.stl, confirmed on
/// the rig): the clips cover ~11 mm from each long edge between 42 and 80 mm
/// below the camera end (≈ y 240–466 pt in portrait, 50 pt each side). So:
/// the info panel lives in the top 240 pt (never covered), the preview takes
/// the rest below it (only its upper side edges can be clipped).
/// "Horizontal" rotates the whole UI 270° (= 90° counter-clockwise) so it
/// reads when the phone lies in the cradle with its camera end to the right;
/// the physical arrangement (panel at the camera end, preview below) stays.
struct ContentView: View {
    @EnvironmentObject var state: AppState
    @AppStorage("uiHorizontal") private var horizontal = false   // vertical (0°) or horizontal (270°)
    @AppStorage("previewFill") private var previewFill = false   // crop to fill (bigger beetle) vs fit
    @AppStorage("showClips") private var showClips = true        // outline the cradle-covered zones
    @AppStorage("stackStart") private var stackStart = "0.0"     // bench stack interval
    @AppStorage("stackEnd") private var stackEnd = "0.5"
    @AppStorage("stackFrames") private var stackFrames = "40"
    private var rotation: Int { horizontal ? 270 : 0 }
    private let panelExtent: CGFloat = 240                       // free zone at the camera end (pt)
    private let hiddenExtent: CGFloat = 226                      // cradle band 240–466 pt: NOTHING useful here
    @State private var lensSlider: Double = 0.25

    var body: some View {
        GeometryReader { geo in
            let sideways = rotation % 180 != 0
            let w = sideways ? geo.size.height : geo.size.width
            let h = sideways ? geo.size.width : geo.size.height
            content(logicalSize: CGSize(width: w, height: h), sideways: sideways)
                .frame(width: w, height: h)
                .rotationEffect(.degrees(Double(rotation)))
                .frame(width: geo.size.width, height: geo.size.height)
                .overlay { if showClips { ClipZones(screen: geo.size) } }
        }
        .ignoresSafeArea()
        .background(Color.black)
    }

    /// videoRotationAngle that keeps the camera image upright after the UI rotation
    private var videoAngle: CGFloat { CGFloat((90 - rotation + 360) % 360) }

    @ViewBuilder
    private func content(logicalSize: CGSize, sideways: Bool) -> some View {
        if sideways {
            // rotation 270°: logical trailing edge = physical top (camera end)
            HStack(spacing: 0) {
                preview(sideways: true)
                Color.black.frame(width: hiddenExtent)               // covered by the cradle
                panel(sideways: true).frame(width: panelExtent)
            }
        } else {
            VStack(spacing: 0) {
                panel(sideways: false).frame(height: panelExtent)
                Color.black.frame(height: hiddenExtent)              // covered by the cradle
                preview(sideways: false)
            }
        }
    }

    private func preview(sideways: Bool) -> some View {
        PreviewView(session: state.camera.session, videoAngle: videoAngle, fill: previewFill)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .clipped()
            .overlay(alignment: .bottomLeading) {
                Text(state.sessionId.map { "session \($0)" } ?? "no session")
                    .font(.system(size: 11, design: .monospaced)).foregroundColor(.yellow).padding(4)
            }
    }

    private func panel(sideways: Bool) -> some View {
        // sideways: logical width = the 240 pt free zone, so everything wraps
        // into short rows; the Dynamic Island sits at the logical trailing
        // edge (physical top) -> keep a margin there.
        VStack(alignment: .leading, spacing: 2) {
            if !sideways { Spacer().frame(height: 34) }             // Dynamic Island (vertical)
            Group {
                row("Server", state.serverStatus)
                row("Cmd", state.lastCommand)
                row("Lens", state.lensPosition.map { String(format: "%.3f", $0) } ?? "-")
                row("Exposure", state.exposureText)
                row("Captured", "\(state.captureCount)")
                row("Thermal", state.thermal)
                row("Free", state.freeText)
            }
            .font(.system(size: 10, design: .monospaced))
            HStack(spacing: 4) {
                Text("lens").foregroundColor(.gray)
                Slider(value: $lensSlider, in: 0...1, onEditingChanged: { editing in
                    if !editing { state.setLens(Float(lensSlider)) }
                }).controlSize(.mini)
                Text(String(format: "%.2f", lensSlider)).frame(width: 34)
            }
            .font(.system(size: 10, design: .monospaced))
            HStack(spacing: 3) {
                Text("stack").foregroundColor(.gray)
                TextField("start", text: $stackStart).frame(width: 40)
                Text("→").foregroundColor(.gray)
                TextField("end", text: $stackEnd).frame(width: 40)
                Text("×").foregroundColor(.gray)
                TextField("n", text: $stackFrames).frame(width: 34)
            }
            .font(.system(size: 11, design: .monospaced))
            .textFieldStyle(.roundedBorder).keyboardType(.decimalPad)
            .foregroundColor(.black)
            HStack(spacing: 3) {
                Button("Shoot") {
                    state.benchShoot(start: Double(stackStart) ?? 0, end: Double(stackEnd) ?? 0.5, frames: Int(stackFrames) ?? 1)
                }.buttonStyle(.borderedProminent).disabled(!state.cameraReady)
                Button(previewFill ? "Fit" : "Fill") { previewFill.toggle() }.buttonStyle(.bordered)
                Button(horizontal ? "Vert" : "Horiz") { horizontal.toggle() }.buttonStyle(.bordered)
                Button(showClips ? "Clips✓" : "Clips") { showClips.toggle() }.buttonStyle(.bordered)
            }
            .font(.system(size: 11))
            .controlSize(.small)
            Text(state.log.last ?? "")
                .font(.system(size: 9, design: .monospaced)).foregroundColor(.gray).lineLimit(1)
        }
        .onChange(of: state.lensPosition) { _, v in if let v { lensSlider = Double(v) } }
        .padding(.leading, 8).padding(.trailing, sideways ? 44 : 8).padding(.vertical, 4)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
        .clipped()
        .background(Color.black)
        .foregroundColor(.white)
    }

    private func row(_ k: String, _ v: String) -> some View {
        HStack { Text(k).foregroundColor(.gray); Spacer(); Text(v).lineLimit(1) }
    }
}

/// Live preview (AVCaptureVideoPreviewLayer). `videoAngle` compensates the
/// UI rotation; `fill` crops the frame to fill the view (bigger subject).
struct PreviewView: UIViewRepresentable {
    let session: AVCaptureSession
    let videoAngle: CGFloat
    let fill: Bool

    final class View: UIView {
        override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
        var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
    }

    func makeUIView(context: Context) -> View {
        let v = View()
        v.previewLayer.session = session
        apply(v)
        return v
    }

    func updateUIView(_ uiView: View, context: Context) { apply(uiView) }

    private func apply(_ v: View) {
        v.previewLayer.videoGravity = fill ? .resizeAspectFill : .resizeAspect
        if let c = v.previewLayer.connection, c.isVideoRotationAngleSupported(videoAngle) {
            c.videoRotationAngle = videoAngle
        }
    }
}

/// The cradle clips (from phone_mount_conf3.stl): ~11 mm in from each long
/// edge, 42–80 mm below the camera end. Drawn in PHYSICAL screen points
/// (iPhone 15 Pro: 393×852 pt over a 70.6×146.6 mm body, 1.55 mm bezel).
struct ClipZones: View {
    let screen: CGSize
    var body: some View {
        // the whole band 42–80 mm below the camera end is unusable (clips at the
        // sides, and the middle is covered too on the real cradle)
        let ptPerMm = screen.height / 143.5
        let y0 = (42.0 - 1.55) * ptPerMm, y1 = (80.0 - 1.55) * ptPerMm
        Rectangle().fill(Color.red.opacity(0.25))
            .frame(width: screen.width, height: y1 - y0)
            .offset(y: y0)
            .frame(width: screen.width, height: screen.height, alignment: .topLeading)
            .allowsHitTesting(false)
    }
}
