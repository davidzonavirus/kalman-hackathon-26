// `swift run gsk-checks` — exit code 0 iff every check passes.

var runner = CheckRunner()
protocolChecks(&runner)
filterChecks(&runner)
flowChecks(&runner)
heightChecks(&runner)

runner.section("PhoneRuntime")
phoneRuntimeChecks(&runner)

runner.finish()
