package frc.engine.utils.Units

@JvmInline
value class VelocityOwO(val asMetersPerSecond: Double) {

}
// constructos
inline val Number.MetersPerSecond get() = VelocityOwO(this.toDouble())
inline val Number.FeetPerSecond get() = VelocityOwO(this.toDouble() * 0.3084)
inline val Number.KilometersPerHour get() = VelocityOwO(this.toDouble() / 3.6)
inline val Number.MilesPerHour get() = VelocityOwO(this.toDouble() * 0.44704)

// decsontrucos
inline val VelocityOwO.asFeetPerSecond get() = asMetersPerSecond / 0.3084
inline val VelocityOwO.asKilometerPerHour get() = asMetersPerSecond * 3.6
inline val VelocityOwO.asMilesPerHour get() = asMetersPerSecond / 0.44704