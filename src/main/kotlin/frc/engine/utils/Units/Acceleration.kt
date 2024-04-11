package frc.engine.utils.Units

import edu.wpi.first.math.Num
import frc.engine.utils.MetersPerSecondSquared

@JvmInline
value class Acceleration (val asMetersPerSecondSquared: Double) { }

// Constructors
inline val Number.metersPerSecondSquared get() = Acceleration(this.toDouble())
inline val Number.`Mps^2` get() = this.metersPerSecondSquared

inline val Number.feetPerSecondSquared get() = Acceleration(this.toDouble() * 0.3048)

// Destructors
inline val Acceleration.asFeetPerSecondSquared get() = asMetersPerSecondSquared / 0.3048