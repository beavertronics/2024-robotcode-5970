package frc.engine.utils.Units.Angular

import edu.wpi.first.math.geometry.Rotation2d
import frc.engine.utils.Sugar.TAU
import frc.engine.utils.Sugar.degreesToRadians
import frc.engine.utils.Sugar.radiansToDegrees
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Time

@JvmInline
value class AngleUnit(val asRadians: Double) {
    operator fun plus(other : AngleUnit) = AngleUnit(asRadians + other.asRadians)
    operator fun minus(other : AngleUnit) = AngleUnit(asRadians - other.asRadians)
    operator fun times(factor : Double) = AngleUnit(asRadians * factor)
    operator fun div(factor: Double) = AngleUnit(asRadians / factor)

    operator fun rem(other: AngleUnit) = AngleUnit(asRadians % other.asRadians)
    operator fun rem(other: Number) = AngleUnit(asRadians % other.toDouble())
    operator fun unaryPlus() = this
    operator fun unaryMinus() = AngleUnit(-asRadians)
    fun getCoterminal() : AngleUnit {
        var coterminalAngle = this % TAU
        if(coterminalAngle.asRadians < 0) return TAU.radians - coterminalAngle
        return coterminalAngle
    }
    fun sin() = AngleUnit.sin(this)
    fun cos() = AngleUnit.cos(this)
    fun tan() = AngleUnit.tan(this)
    fun csc() = AngleUnit.csc(this)
    fun sec() = AngleUnit.sec(this)
    fun cot() = AngleUnit.tan(this)



    // Unit conversion math
    operator fun div(other : Time) = AngularVelocity(asRadians/other.asSeconds)
    operator fun times(other : DistanceUnit) = DistanceUnit(asRadians/other.asMeters)

    /**
     * Taken from mean machines mean lib
     */
    companion object {
        @JvmStatic
        fun sin(angle: AngleUnit) = kotlin.math.sin(angle.asRadians)

        @JvmStatic
        fun cos(angle: AngleUnit) = kotlin.math.cos(angle.asRadians)

        @JvmStatic
        fun tan(angle: AngleUnit) = kotlin.math.tan(angle.asRadians)
        @JvmStatic
        fun csc(angle: AngleUnit) =  1 / kotlin.math.sin(angle.asRadians)

        @JvmStatic
        fun sec(angle: AngleUnit) = 1 / kotlin.math.cos(angle.asRadians)

        @JvmStatic
        fun cot(angle: AngleUnit) = kotlin.math.cos(angle.asRadians) / kotlin.math.sin(angle.asRadians)


        @JvmStatic
        fun asin(value: Double) = AngleUnit(kotlin.math.asin(value))

        @JvmStatic
        fun acos(value: Double) = AngleUnit(kotlin.math.acos(value))

        @JvmStatic
        fun atan(value: Double) = AngleUnit(kotlin.math.atan(value))

        @JvmStatic
        fun atan2(y: Double, x: Double) = AngleUnit(kotlin.math.atan2(y, x))

        @JvmStatic
        fun atan2(y: DistanceUnit, x: DistanceUnit) = AngleUnit(Math.toDegrees(
            kotlin.math.atan2(
                y.asMeters,
                x.asMeters
            )
        ))
    }
}

inline val Rotation2d.beaverRadians get() = AngleUnit(this.radians)

// constructors
inline val Number.rotations get() = AngleUnit(this.toDouble() * TAU)
inline val Number.radians get() = AngleUnit(this.toDouble())
inline val Number.degrees get() = AngleUnit(this.toDouble().degreesToRadians())

// destructors
inline val AngleUnit.asRotations get() = asRadians / TAU
inline val AngleUnit.asDegrees get() = asRadians.radiansToDegrees()