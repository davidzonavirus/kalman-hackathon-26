import Foundation

/// CRC-16 helpers used by the telemetry wire format.
public enum CRC16 {
    /// Precomputed table for poly 0x1021 (MSB-first, non-reflected).
    private static let table: [UInt16] = (0..<256).map { i -> UInt16 in
        var c = UInt16(i) << 8
        for _ in 0..<8 {
            c = (c & 0x8000) != 0 ? (c << 1) ^ 0x1021 : (c << 1)
        }
        return c
    }

    /// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0.
    /// Check value: `ccittFalse(Array("123456789".utf8)) == 0x29B1`.
    public static func ccittFalse<S: Sequence>(_ bytes: S) -> UInt16 where S.Element == UInt8 {
        var crc: UInt16 = 0xFFFF
        for b in bytes {
            let idx = Int(((crc >> 8) ^ UInt16(b)) & 0xFF)
            crc = (crc << 8) ^ table[idx]
        }
        return crc
    }
}
