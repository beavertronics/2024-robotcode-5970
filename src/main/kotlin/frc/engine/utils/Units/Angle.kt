package frc.engine.utils.Units

import frc.engine.utils.Sugar.radiansToDegrees

@JvmInline
value class Angle(val asRadians: Double) { }
// constructors
inline val Number.radians get() = Angle(Math.toDegrees(this.toDouble()))
inline val Number.degrees get() = Angle(this.toDouble())

// destructors
inline val Angle.asRadians get() = asRadians
inline val Angle.asDegrees get() = asRadians.radiansToDegrees()