/// Name -> factory for every filter variant. The app's picker, `meta.json` "filter_name"
/// and `kfreplay --filter` all go through here.
public enum FilterRegistry {
    public static let all: [String: @Sendable () -> any GroundSpeedFilter] = [
        ReferenceKF4.name: { ReferenceKF4() },
        DecoupledKF2x2.name: { DecoupledKF2x2() },
        // Register new GroundSpeedFilter implementations here: MyFilter.name: { MyFilter() },
    ]

    /// Filter used when nothing else is selected.
    public static let defaultName = ReferenceKF4.name

    /// Sorted names, for pickers.
    public static var names: [String] { all.keys.sorted() }

    /// New instance of the named filter (nil if unknown), optionally with a config.
    public static func make(name: String) -> (any GroundSpeedFilter)? {
        all[name]?()
    }

    public static func make(name: String, config: FilterConfig) -> (any GroundSpeedFilter)? {
        guard let f = make(name: name) else { return nil }
        f.config = config
        return f
    }
}
