import SwiftUI
import UIKit

/// Datasheet aesthetic: warm paper, graphite ink, warm-grey hairlines, one restrained
/// "live" accent (deep teal) and one alert colour (burnt vermilion) used only for faults
/// and the recording ring. Light = bone paper, dark = warm charcoal (follows the system).
enum Theme {
    static let paper    = dynamic(light: 0xF3EFE7, dark: 0x1E1C19)   // background
    static let surface  = dynamic(light: 0xFBF9F4, dark: 0x262320)   // raised rows / sheets
    static let ink      = dynamic(light: 0x2B2824, dark: 0xECE6DA)   // primary text, record key
    static let inkSoft  = dynamic(light: 0x6E675D, dark: 0xA39B8E)   // secondary text
    static let inkFaint = dynamic(light: 0xA39B8E, dark: 0x6E675D)   // tertiary, inactive
    static let hairline = dynamic(light: 0xDDD6CA, dark: 0x3A362F)   // rules
    static let live     = dynamic(light: 0x2F6B66, dark: 0x5FA39C)   // ok / live (deep teal)
    static let alert    = dynamic(light: 0xC2462B, dark: 0xE0643F)   // faults (burnt vermilion)
    /// Text drawn on top of `ink` (record key label).
    static let onInk    = dynamic(light: 0xF3EFE7, dark: 0x1E1C19)

    // MARK: Type (all ship with iOS; Font.custom falls back to the system font)

    /// Serif for the few words on screen.
    static func serif(_ size: CGFloat, bold: Bool = false) -> Font {
        .custom(bold ? "Charter-Bold" : "Charter-Roman", size: size, relativeTo: .body)
    }
    /// Every number.
    static func mono(_ size: CGFloat, bold: Bool = false) -> Font {
        .custom(bold ? "Menlo-Bold" : "Menlo-Regular", size: size, relativeTo: .body).monospacedDigit()
    }

    private static func dynamic(light: UInt32, dark: UInt32) -> Color {
        Color(UIColor { traits in
            UIColor(hex: traits.userInterfaceStyle == .dark ? dark : light)
        })
    }
}

extension UIColor {
    convenience init(hex: UInt32) {
        self.init(red: CGFloat((hex >> 16) & 0xFF) / 255,
                  green: CGFloat((hex >> 8) & 0xFF) / 255,
                  blue: CGFloat(hex & 0xFF) / 255,
                  alpha: 1)
    }
}

/// Letter-spaced small uppercase label ("V_X", "DISTANCE").
struct Micro: View {
    let text: String
    var color: Color = Theme.inkSoft
    init(_ text: String, color: Color = Theme.inkSoft) {
        self.text = text
        self.color = color
    }
    var body: some View {
        Text(text.uppercased())
            .font(.custom("Menlo-Regular", size: 9, relativeTo: .caption2))
            .tracking(1.8)
            .foregroundStyle(color)
    }
}

/// A full-width warm-grey rule.
struct Hairline: View {
    var body: some View {
        Rectangle().fill(Theme.hairline).frame(height: 1)
    }
}

/// One gentle staggered appear.
struct Reveal: ViewModifier {
    let index: Int
    let shown: Bool
    func body(content: Content) -> some View {
        content
            .opacity(shown ? 1 : 0)
            .offset(y: shown ? 0 : 8)
            .animation(.easeOut(duration: 0.5).delay(0.08 * Double(index)), value: shown)
    }
}

extension View {
    func reveal(_ index: Int, _ shown: Bool) -> some View { modifier(Reveal(index: index, shown: shown)) }
}
