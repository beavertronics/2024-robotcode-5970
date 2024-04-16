package frc.engine.utils.geometry
// File adapted from 2898 2023 bpsrobotics engine
import frc.engine.utils.Sugar.eqEpsilon
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Angle
import frc.engine.utils.Units.Angular.AngleUnit
import frc.engine.utils.Units.Angular.radians
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Linear.meters
import frc.engine.utils.geometry.Coordinate
import kotlin.math.*

class Line(val point1 : Coordinate, val point2 : Coordinate){
    fun intersects(coordinate : Coordinate, theta : AngleUnit) : Boolean{
        val theta1 = atan2((point1.y-coordinate.y).asMeters, (point1.x-coordinate.x).asMeters)
        val theta2 = atan2((point2.y-coordinate.y).asMeters, (point2.x-coordinate.x).asMeters)
        var dtheta = theta2-theta1
        var ntheta=theta.asRadians-theta1
        dtheta= atan2(sin(dtheta), cos(dtheta))
        ntheta= atan2(sin(ntheta), cos(ntheta))
        return (sign(ntheta) == sign(dtheta) && abs(ntheta) <= abs(dtheta))
    }
    /**
     * Gets the intersection of a ray-cast and the line
     * @param coordinate Origin of the ray-cast
     * @param theta Rotation of the ray-cast given in radians
     * @return Intersection point of ray-cast and line
     * @sample intersection
     * */
    fun intersection(coordinate : Coordinate, theta : AngleUnit) : Coordinate? {
        val xPosition : DistanceUnit
        val yPosition : DistanceUnit
        if(!intersects(coordinate, theta)) return null
        println(theta.cos() eqEpsilon 0)
        if(point2.x-point1.x == 0.0.meters || theta.cos() eqEpsilon 0){
            val cotOfTheta = theta.cot()
            var slope = (point2.x-point1.x)/(point2.y-point1.y).asMeters
            if(slope.asMeters.isInfinite()) {
                yPosition=point1.y
            }
            else {
                yPosition=(point1.x-slope*point1.y.asMeters-coordinate.x + coordinate.y*cotOfTheta)/(cotOfTheta-slope.asMeters)
            }
            xPosition =yPosition*cotOfTheta+coordinate.x-coordinate.y*cotOfTheta
        }else{
            val rm = theta.tan()
            val lm = (point2.y-point1.y).asMeters/(point2.x-point1.x).asMeters
            xPosition= (point1.y-point1.x * lm-coordinate.y+coordinate.x * rm)/(rm-lm)
            yPosition=xPosition*rm+coordinate.y-coordinate.x*rm
        }
        return Coordinate(xPosition, yPosition)
    }
    /**
     * Gets the intersection of a ray-cast and the line
     * @param pose Pose of the ray-cast
     * @return Intersection point of ray-cast and line
     * @sample intersection
     * */
    fun intersection(pose: Pose2d) : Coordinate? {
        return intersection(Coordinate(pose.x.meters,pose.y.meters), pose.rotation.radians.radians)
    }
    /**
     * Gets the distance from the intersection of a ray-cast and the line
     * @param coordinate Origin of the ray-cast
     * @param rotation Rotation of the raycast
     * @return Distance from the intersection point of ray-cast and line
     * @sample distance
     * */
    fun distance(coordinate : Coordinate, rotation : AngleUnit) : DistanceUnit? {
        val intersectionPoint = intersection(coordinate, rotation) ?: return null
        return (coordinate - intersectionPoint).magnitude
    }
    /**
     * Gets the distance from the intersection of a ray-cast and the line
     * @param pose Pose of the raycast
     * @return Distance from the intersection point of ray-cast and line
     * @sample distance
     * */
    fun distance(pose : Pose2d) : DistanceUnit? {
        val intersectionPoint = intersection(pose) ?: return null
        return (Coordinate(pose.x.meters,pose.y.meters) - intersectionPoint).magnitude
    }
    /**
     * Returns the line reflected over a vertical line at the given x coordinate
     * @param x X value the relection line
     * @return Reflected line
     * @author Ozy King
     */
    fun reflectHorizontally(x: DistanceUnit) : Line {
        return Line(point1.reflectHorizontally(x),point2.reflectHorizontally(x))
    }
}