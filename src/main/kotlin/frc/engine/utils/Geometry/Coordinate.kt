package frc.engine.utils.geometry
// File adapted from 2898's bpsrobotics engine 
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Linear.meters
import kotlin.math.pow
import kotlin.math.sqrt

class Coordinate(val x: DistanceUnit, val y: DistanceUnit) {
    constructor(pose: Pose2d) : this(pose.x.meters, pose.y.meters)
    /**
     * Distance from 0, 0, calculated using pythagorean theorem
     * */
    val magnitude get() = DistanceUnit(sqrt(x.asMeters.pow(2) + y.asMeters.pow(2)))
    fun distance(pos: Coordinate): DistanceUnit  { return DistanceUnit(sqrt((x-pos.x).asMeters.pow(2) + (y-pos.y).asMeters.pow(2))) }
    fun distance(pose: Pose2d): DistanceUnit     { return distance(Coordinate(pose)) }
    fun xdistance(pos: DistanceUnit): DistanceUnit     { return x-pos}
    fun xdistance(pos: Coordinate): DistanceUnit { return x-pos.x}
    fun xdistance(pos: Pose2d): DistanceUnit     { return xdistance(Coordinate(pos))}
    fun ydistance(pos: DistanceUnit): DistanceUnit     { return y-pos}
    fun ydistance(pos: Coordinate): DistanceUnit { return y-pos.y}
    fun ydistance(pos: Pose2d): DistanceUnit     { return ydistance(Coordinate(pos))}

    operator fun plus(other: Coordinate) : Coordinate {
        return Coordinate(x + other.x, y + other.y)
    }
    operator fun minus(other: Coordinate) : Coordinate {
        return Coordinate(x - other.x,y - other.y)
    }
    operator fun div(other: Double) : Coordinate{
        return Coordinate(x / other,y / other)
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
    fun reflectHorizontally(x: DistanceUnit) : Coordinate{
        return Coordinate(x + (x - this.x),y)
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