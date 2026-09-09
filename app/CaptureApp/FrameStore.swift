import Foundation

/// Writes DNG + sidecar into Documents/<session>/ atomically. Never deletes.
final class FrameStore {
    var session: String?
    private let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]

    func sessionDir(_ session: String) throws -> URL {
        let d = docs.appendingPathComponent(session, isDirectory: true)
        try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
        return d
    }

    func write(session: String, name: String, dng: Data, sidecar: [String: Any]) throws {
        let d = try sessionDir(session)
        try atomicWrite(dng, to: d.appendingPathComponent(name + ".dng"))
        let json = try JSONSerialization.data(withJSONObject: FrameStore.jsonSafe(sidecar), options: [.prettyPrinted, .sortedKeys])
        try atomicWrite(json, to: d.appendingPathComponent(name + ".json"))
    }

    private func atomicWrite(_ data: Data, to url: URL) throws {
        let tmp = url.appendingPathExtension("part")
        try data.write(to: tmp, options: .atomic)
        _ = try? FileManager.default.removeItem(at: url)
        try FileManager.default.moveItem(at: tmp, to: url)
    }

    func freeBytes() -> Int64 {
        (try? docs.resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey]).volumeAvailableCapacityForImportantUsage) ?? 0
    }

    /// Replace non-JSON values (NSNull stays, Float/Int32 -> Double/Int, NaN -> null).
    static func jsonSafe(_ v: Any) -> Any {
        switch v {
        case let d as [String: Any]: return d.mapValues { jsonSafe($0) }
        case let a as [Any]: return a.map { jsonSafe($0) }
        case let f as Float: return f.isFinite ? Double(f) : NSNull()
        case let d as Double: return d.isFinite ? d : NSNull()
        case let i as Int32: return Int(i)
        case let i as Int64: return Int(i)
        case let g as CGFloat: return Double(g)
        case is NSNull: return NSNull()
        case Optional<Any>.none: return NSNull()
        default: return v
        }
    }
}
