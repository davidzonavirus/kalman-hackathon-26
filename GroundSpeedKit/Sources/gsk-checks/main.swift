// `swift run gsk-checks` — exit code 0 iff every check passes.

var runner = CheckRunner()
protocolChecks(&runner)
filterChecks(&runner)
flowChecks(&runner)
heightChecks(&runner)

// MARK: registerPhoneRuntimeChecks — Agent B's checks live in PhoneRuntimeChecks.swift.
runner.section("PhoneRuntime")
phoneRuntimeChecks(&runner)

runner.finish()
