import Foundation

/// Minimal test harness (no XCTest with Command Line Tools). Prints PASS/FAIL per check.
struct CheckRunner {
    private(set) var passed = 0
    private(set) var failed: [String] = []

    mutating func section(_ name: String) {
        print("\n== \(name) ==")
    }

    mutating func check(_ name: String, _ ok: Bool, _ detail: @autoclosure () -> String = "") {
        let d = detail()
        if ok {
            passed += 1
            print("PASS  \(name)\(d.isEmpty ? "" : "  [\(d)]")")
        } else {
            failed.append(name)
            print("FAIL  \(name)\(d.isEmpty ? "" : "  [\(d)]")")
        }
    }

    mutating func near(_ name: String, _ value: Double, _ expected: Double, tol: Double) {
        check(name, abs(value - expected) <= tol,
              String(format: "got %.6g, expected %.6g ± %.3g", value, expected, tol))
    }

    /// Runs `body`, failing the check if it throws.
    mutating func noThrow(_ name: String, _ body: () throws -> Void) {
        do { try body() } catch { check(name, false, "threw \(error)") }
    }

    func finish() -> Never {
        print("\n\(passed) passed, \(failed.count) failed")
        if !failed.isEmpty {
            print("Failed: " + failed.joined(separator: ", "))
            exit(1)
        }
        exit(0)
    }
}
