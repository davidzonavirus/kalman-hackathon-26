import Foundation
import Network
import SpeedProtocol

/// Handles one parsed command and returns the reply. Called on the server's queue;
/// implementations must be thread-safe (or hop to their own queue synchronously).
public protocol CommandHandling: AnyObject {
    func handle(_ command: Command) -> CommandReply
}

/// TCP command server (PROTOCOL.md §2): listens on 9001, advertises Bonjour
/// `_groundspeed._tcp`, newline-delimited JSON in both directions, one reply per request.
public final class CommandServer: @unchecked Sendable {
    public static let defaultPort: UInt16 = 9001
    public static let bonjourType = "_groundspeed._tcp"

    public weak var handler: CommandHandling?
    /// Called (on the server queue) when a dashboard connects; argument is the peer's IP
    /// (IPv4-mapped IPv6 addresses are unwrapped to dotted IPv4). Use it to retarget UDP.
    public var onPeerConnected: (@Sendable (String) -> Void)?
    /// Called whenever the number of connected clients changes.
    public var onClientCountChanged: (@Sendable (Int) -> Void)?

    public let port: UInt16
    public let bonjourName: String?
    private let queue = DispatchQueue(label: "gsk.commands", qos: .userInitiated)
    /// Serial: replies stay in request order.
    private let handlerQueue = DispatchQueue(label: "gsk.commands.handler", qos: .userInitiated)
    private var listener: NWListener?
    private var clients: [ObjectIdentifier: Client] = [:]
    private var _state: String = "stopped"

    private final class Client {
        let connection: NWConnection
        var lines = LineBuffer()
        init(_ c: NWConnection) { connection = c }
    }

    public init(port: UInt16 = CommandServer.defaultPort, bonjourName: String? = nil) {
        self.port = port
        self.bonjourName = bonjourName
    }

    public var state: String { queue.sync { _state } }
    public var clientCount: Int { queue.sync { clients.count } }

    private var wantRunning = false

    public func start() throws {
        let l = try makeListener()
        queue.sync {
            self.wantRunning = true
            self.listener = l
        }
        l.start(queue: queue)
    }

    /// iOS fails the listener when the app is backgrounded or the network changes
    /// (hotspot drop/rejoin) and never revives it, so a failed listener is rebuilt.
    private func makeListener() throws -> NWListener {
        guard let p = NWEndpoint.Port(rawValue: port) else { throw CommandServerError.badPort }
        let params = NWParameters.tcp
        params.allowLocalEndpointReuse = true
        let l = try NWListener(using: params, on: p)
        l.service = NWListener.Service(name: bonjourName, type: Self.bonjourType)
        l.stateUpdateHandler = { [weak self, weak l] st in
            guard let self else { return }
            switch st {
            case .ready: self._state = "listening:\(self.port)"
            case .failed(let e):
                self._state = "failed: \(e)"
                l?.cancel()
                self.scheduleRestart()
            case .waiting(let e): self._state = "waiting: \(e)"
            case .cancelled: if !self.wantRunning { self._state = "stopped" }
            default: break
            }
        }
        l.newConnectionHandler = { [weak self] conn in
            self?.accept(conn)
        }
        return l
    }

    private func scheduleRestart() {
        queue.asyncAfter(deadline: .now() + 1) { [weak self] in
            guard let self, self.wantRunning else { return }
            self.listener?.cancel()
            guard let l = try? self.makeListener() else { self.scheduleRestart(); return }
            self.listener = l
            l.start(queue: self.queue)
        }
    }

    /// Rebuilds the listener unless it is listening (call when the app returns to foreground).
    public func restartIfNeeded() {
        queue.async { [weak self] in
            guard let self, self.wantRunning, !self._state.hasPrefix("listening") else { return }
            self.scheduleRestart()
        }
    }

    public func stop() {
        queue.sync {
            wantRunning = false
            listener?.cancel()
            listener = nil
            for (_, c) in clients { c.connection.cancel() }
            clients.removeAll()
        }
    }

    /// Blocks until the listener is ready or `timeout` elapses. Returns true when ready.
    public func waitUntilReady(timeout: TimeInterval = 2.0) -> Bool {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if state.hasPrefix("listening") { return true }
            Thread.sleep(forTimeInterval: 0.01)
        }
        return false
    }

    // MARK: Internals (queue only)

    private func accept(_ conn: NWConnection) {
        let client = Client(conn)
        let id = ObjectIdentifier(conn)
        clients[id] = client
        onClientCountChanged?(clients.count)
        conn.stateUpdateHandler = { [weak self, weak conn] st in
            guard let self, let conn else { return }
            switch st {
            case .ready:
                if let ip = Self.peerIP(conn.endpoint) { self.onPeerConnected?(ip) }
            case .failed, .cancelled:
                self.drop(conn)
            default: break
            }
        }
        conn.start(queue: queue)
        receive(on: conn)
    }

    private func drop(_ conn: NWConnection) {
        let id = ObjectIdentifier(conn)
        if clients.removeValue(forKey: id) != nil {
            conn.cancel()
            onClientCountChanged?(clients.count)
        }
    }

    private func receive(on conn: NWConnection) {
        conn.receive(minimumIncompleteLength: 1, maximumLength: 64 * 1024) { [weak self, weak conn] data, _, isComplete, error in
            guard let self, let conn else { return }
            if let data, !data.isEmpty, let client = self.clients[ObjectIdentifier(conn)] {
                let lines = client.lines.append(data)
                for line in lines {
                    // Handlers may block for seconds (measure_height); doing that on `queue`
                    // would stall `state`/`clientCount`, which the UI reads with queue.sync.
                    self.handlerQueue.async {
                        let reply = self.process(line)
                        conn.send(content: reply.encodeLine(), completion: .contentProcessed { _ in })
                    }
                }
            }
            if isComplete || error != nil {
                self.drop(conn)
                return
            }
            self.receive(on: conn)
        }
    }

    private func process(_ line: Data) -> CommandReply {
        let cmd: Command
        do {
            cmd = try Command.parse(line)
        } catch {
            let name = (try? JSONSerialization.jsonObject(with: line) as? [String: Any])?["cmd"] as? String ?? "?"
            return .failure(name, "parse error: \(error)")
        }
        guard let handler else { return .failure(cmd.name, "no handler") }
        return handler.handle(cmd)
    }

    /// Extracts a dotted/colon IP string from an endpoint, unwrapping IPv4-mapped IPv6 and
    /// stripping any `%iface` scope suffix.
    public static func peerIP(_ endpoint: NWEndpoint) -> String? {
        guard case let .hostPort(host, _) = endpoint else { return nil }
        switch host {
        case .ipv4(let a):
            return Self.stripScope("\(a)")
        case .ipv6(let a):
            if let v4 = a.asIPv4 { return Self.stripScope("\(v4)") }
            return Self.stripScope("\(a)")
        case .name(let n, _):
            return n
        @unknown default:
            return nil
        }
    }

    private static func stripScope(_ s: String) -> String {
        if let i = s.firstIndex(of: "%") { return String(s[..<i]) }
        return s
    }
}

public enum CommandServerError: Error {
    case badPort
}
