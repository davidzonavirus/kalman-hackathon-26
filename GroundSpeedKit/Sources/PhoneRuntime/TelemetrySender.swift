import Foundation
import Network
import SpeedProtocol

/// Streams `TelemetryFrame`s over UDP (PROTOCOL.md §1) at a fixed rate (default 50 Hz).
///
/// The sender pulls the latest frame from `frameSource` on its own timer, stamps the
/// sequence number, and sends it binary (default) or as a JSON debug datagram.
/// The destination can be changed at any time (e.g. when a dashboard connects over TCP).
public final class TelemetrySender: @unchecked Sendable {
    public static let defaultPort: UInt16 = 9000

    /// Returns the frame to send (seq is overwritten by the sender). nil = skip this tick.
    public var frameSource: (@Sendable () -> TelemetryFrame?)?

    private let queue = DispatchQueue(label: "gsk.telemetry", qos: .userInitiated)
    private var connection: NWConnection?
    private var timer: DispatchSourceTimer?
    private var seq: UInt32 = 0
    private var _host: String
    private var _port: UInt16
    private var _useJSON: Bool
    private var _rateHz: Double
    private var _framesSent: UInt64 = 0
    private var _sendErrors: UInt64 = 0
    private var _lastError: String?
    private var _connectionReady = false

    public init(host: String, port: UInt16 = TelemetrySender.defaultPort, rateHz: Double = 50, useJSON: Bool = false) {
        _host = host
        _port = port
        _rateHz = rateHz
        _useJSON = useJSON
    }

    deinit {
        timer?.cancel()
        connection?.cancel()
    }

    // MARK: Thread-safe accessors

    public var host: String { queue.sync { _host } }
    public var port: UInt16 { queue.sync { _port } }
    public var framesSent: UInt64 { queue.sync { _framesSent } }
    public var sendErrors: UInt64 { queue.sync { _sendErrors } }
    public var lastError: String? { queue.sync { _lastError } }
    public var isReady: Bool { queue.sync { _connectionReady } }
    public var isRunning: Bool { queue.sync { timer != nil } }

    public var useJSON: Bool {
        get { queue.sync { _useJSON } }
        set { queue.async { self._useJSON = newValue } }
    }

    /// Change the destination. Empty host stops sending until a host is set.
    public func setDestination(host: String, port: UInt16? = nil) {
        queue.async {
            let newPort = port ?? self._port
            guard host != self._host || newPort != self._port || self.connection == nil else { return }
            self._host = host
            self._port = newPort
            if self.timer != nil { self.reconnectLocked() }
        }
    }

    public func start() {
        queue.async {
            guard self.timer == nil else { return }
            self.reconnectLocked()
            let t = DispatchSource.makeTimerSource(flags: .strict, queue: self.queue)
            let interval = 1.0 / max(1.0, self._rateHz)
            t.schedule(deadline: .now() + interval, repeating: interval, leeway: .milliseconds(1))
            weak let me = self
            t.setEventHandler { me?.tickLocked() }
            t.resume()
            self.timer = t
        }
    }

    public func stop() {
        queue.sync {
            self.timer?.cancel()
            self.timer = nil
            self.connection?.cancel()
            self.connection = nil
            self._connectionReady = false
        }
    }

    /// Sends one frame immediately (outside the timer). Seq is stamped by the sender.
    public func sendNow(_ frame: TelemetryFrame) {
        queue.async { self.sendLocked(frame) }
    }

    // MARK: Internals (queue only)

    private func reconnectLocked() {
        connection?.cancel()
        connection = nil
        _connectionReady = false
        let h = _host.trimmingCharacters(in: .whitespaces)
        guard !h.isEmpty, let p = NWEndpoint.Port(rawValue: _port) else { return }
        let params = NWParameters.udp
        params.serviceClass = .interactiveVideo   // low-latency
        let c = NWConnection(host: NWEndpoint.Host(h), port: p, using: params)
        c.stateUpdateHandler = { [weak self, weak c] state in
            guard let self else { return }
            // Runs on self.queue.
            switch state {
            case .ready:
                self._connectionReady = true
                self._lastError = nil
            case .failed(let err):
                self._connectionReady = false
                self._lastError = "\(err)"
                // Retry shortly (e.g. network came back / hotspot joined).
                if let c, c === self.connection {
                    self.queue.asyncAfter(deadline: .now() + 1.0) { [weak self] in
                        guard let self, self.timer != nil, c === self.connection else { return }
                        self.reconnectLocked()
                    }
                }
            case .waiting(let err):
                self._connectionReady = false
                self._lastError = "\(err)"
            case .cancelled:
                self._connectionReady = false
            default:
                break
            }
        }
        connection = c
        c.start(queue: queue)
    }

    private func tickLocked() {
        guard let frame = frameSource?() else { return }
        sendLocked(frame)
    }

    private func sendLocked(_ frame: TelemetryFrame) {
        guard let c = connection, _connectionReady else { return }
        var f = frame
        f.seq = seq
        seq &+= 1
        let payload = _useJSON ? f.encodeJSON() : f.encodeBinary()
        c.send(content: payload, completion: .contentProcessed { [weak self] err in
            guard let self else { return }
            if let err {
                self._sendErrors &+= 1
                self._lastError = "\(err)"
            } else {
                self._framesSent &+= 1
            }
        })
    }
}
