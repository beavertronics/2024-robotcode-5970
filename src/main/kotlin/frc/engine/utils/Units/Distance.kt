package frc.engine.utils.Units
// this is mine now from Mean Machine (2471)
@JvmInline
value class Distance(val asMeters: Double) {
    // espagueti
    // uwu
}
// construct your mother
inline val Number.meters get() = Distance(this.toDouble())
inline val Number.feet get() = Distance(this.toDouble() * 0.3048)
inline val Number.inches get() = Distance(this.toDouble() * 0.0254)
// destruct your mother

inline val Distance.asFeet get() = asMeters / 0.3048
inline val Distance.asInches get() = asMeters / 0.0254
