package frc.engine.utils.geometry
// File adapted from 2898's bpsrobotics engine 
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.engine.utils.Units.Angular.AngleUnit
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Linear.meters
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class Vector2(val x: Double, val y: Double) {
    constructor(pose: Pose2d) : this(pose.x, pose.y)
    /**
     * Distance from 0, 0, calculated using pythagorean theorem
     * */
    val magnitude get() = sqrt(x.pow(2) + y.pow(2))
    fun angle()  = AngleUnit(atan2(y,x))
    fun angleTo(other : Vector2) = (this-other).angle()
    fun distance(other: Vector2): Double = (this-other).magnitude
    fun distance(pose: Pose2d): Double     { return distance(Vector2(pose)) }
    fun xdistance(pos: Double): Double     { return x-pos}
    fun xdistance(pos: Vector2): Double { return x-pos.x}
    fun xdistance(pos: Pose2d): Double     { return x - pos.x}
    fun ydistance(pos: Double): Double     { return y-pos}
    fun ydistance(pos: Vector2): Double { return y-pos.y}
    fun ydistance(pos: Pose2d): Double     { return y - pos.y}

    operator fun plus(other: Vector2) : Vector2 {
        return Vector2(x + other.x, y + other.y)
    }
    operator fun minus(other: Vector2) : Vector2 {
        return Vector2(x - other.x,y - other.y)
    }
    operator fun times(other: Double) : Vector2{
        return Vector2(x * other,y * other)
    }
    operator fun div(other: Double) : Vector2{
        return Vector2(x / other,y / other)
    }
    override fun toString(): String {
        return "(x: ${x}, y: ${y})"
    }

    /**
     * Reflects the point across a horizontal line
     * @return Reflected point
     * @param x The x value of the vertical line to reflect across
     * @author Ozy King
     */
    fun reflectHorizontally(x: DistanceUnit) : Vector2{
        return Vector2(x + (x - this.x),y)
    }

    /**
     * Creates a new Pose2d from the coordinate object and rotation
     * @param rotation The rotation of the pose, in degrees
     * @return A new Pose2d contructed from the coordinate and the rotation
     */
    fun toPose2d(rotation: Double): Pose2d{
        return Pose2d(x.asMeters, y.asMeters, Rotation2d.fromDegrees(rotation))
    }
}