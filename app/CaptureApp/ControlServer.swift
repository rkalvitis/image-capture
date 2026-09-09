import Foundation
import Network

protocol ControlHandler: AnyObject {
    /// Handle one command; may call `send` for intermediate messages; returns the final reply.
    func handle(_ msg: [String: Any], send: @escaping ([String: Any]) -> Void) -> [String: Any]
}

/// TCP server, newline-delimited JSON, one command at a time per connection.
/// Reached over the USB tunnel (`pymobiledevice3 usbmux forward 5601 5601`)
/// or Wi-Fi; commands are executed sequentially on a serial queue.
final class ControlServer {
    private let port: UInt16
    private weak var handler: ControlHandler?
    private var listener: NWListener?
    private let queue = DispatchQueue(label: "capture.control")
    private let work = DispatchQueue(label: "capture.commands")   // serial: one command at a time
    private var connections: [ObjectIdentifier: NWConnection] = [:]
    var onStatus: ((String) -> Void)?

    init(port: UInt16, handler: ControlHandler) {
        self.port = port
        self.handler = handler
    }

    func start() {
        do {
            let params = NWParameters.tcp
            params.allowLocalEndpointReuse = true
            listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
        } catch {
            onStatus?("listen failed: \(error)")
            return
        }
        listener?.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready: self?.onStatus?("listening :\(self?.port ?? 0)")
            case .failed(let e): self?.onStatus?("failed: \(e)"); self?.restartLater()
            case .cancelled: self?.onStatus?("cancelled")
            default: break
            }
        }
        listener?.newConnectionHandler = { [weak self] conn in self?.accept(conn) }
        listener?.start(queue: queue)
    }

    private func restartLater() {
        queue.asyncAfter(deadline: .now() + 2) { [weak self] in self?.start() }
    }

    private func accept(_ conn: NWConnection) {
        let id = ObjectIdentifier(conn)
        connections[id] = conn
        conn.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready: self?.onStatus?("connected")
            case .failed, .cancelled:
                self?.connections[id] = nil
                self?.onStatus?("listening :\(self?.port ?? 0)")
            default: break
            }
        }
        conn.start(queue: queue)
        receive(conn, buffer: Data())
    }

    private func receive(_ conn: NWConnection, buffer: Data) {
        conn.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, isComplete, error in
            guard let self else { return }
            var buf = buffer
            if let data { buf.append(data) }
            // split complete lines
            while let nl = buf.firstIndex(of: 0x0A) {
                let line = buf.subdata(in: buf.startIndex..<nl)
                buf.removeSubrange(buf.startIndex...nl)
                if line.isEmpty { continue }
                self.dispatch(line, on: conn)
            }
            if isComplete || error != nil {
                conn.cancel()
                return
            }
            self.receive(conn, buffer: buf)
        }
    }

    private func dispatch(_ line: Data, on conn: NWConnection) {
        guard let obj = try? JSONSerialization.jsonObject(with: line), let msg = obj as? [String: Any] else {
            send(["type": "error", "reason": "malformed JSON"], to: conn)
            return
        }
        work.async { [weak self] in
            guard let self, let handler = self.handler else { return }
            let reply = handler.handle(msg) { [weak self] m in self?.send(m, to: conn) }
            self.send(reply, to: conn)
        }
    }

    private func send(_ obj: [String: Any], to conn: NWConnection) {
        guard var data = try? JSONSerialization.data(withJSONObject: obj) else { return }
        data.append(0x0A)
        conn.send(content: data, completion: .contentProcessed { _ in })
    }
}
