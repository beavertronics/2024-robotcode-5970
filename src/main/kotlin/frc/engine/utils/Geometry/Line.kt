package frc.engine.utils.geometry
// File adapted from 2898 2023 bpsrobotics engine
import frc.engine.utils.Sugar.eqEpsilon
import edu.wpi.first.math.geometry.Pose2d
import frc.engine.utils.Units.Angular.AngleUnit
import frc.engine.utils.Units.Angular.radians
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Linear.meters
import kotlin.math.*

class Line(val point1 : Vector2, val point2 : Vector2){
    val slope get() = (point2.x-point1.x)/(point2.y-point1.y)
    fun intersects(raycast : Raycast2D) : Boolean{
        val theta1 = raycast.origin.angleTo(point1).getCoterminal()
        val theta2 = raycast.origin.angleTo(point2).getCoterminal()
        return min(theta1.asRadians, theta2.asRadians) <= raycast.angle.asRadians && raycast.angle.asRadians <= max(theta1.asRadians, theta2.asRadians)
    }
    /**
     * Gets the intersection of a ray-cast and the line
     * @param coordinate Origin of the ray-cast
     * @param theta Rotation of the ray-cast given in radians
     * @return Intersection point of ray-cast and line
     * @sample intersection
     * */
    fun intersection(raycast : Raycast2D) : Vector2? {
        val xPosition : Double
        val yPosition : Double
        if(!intersects(raycast)) return null
        //println(theta.cos() eqEpsilon 0)
        if(point2.x == point1.x || raycast.angle.cos() eqEpsilon 0){
            val cotOfTheta = raycast.angle.cot()
            if(slope.isInfinite()) {
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
        return Vector2(xPosition, yPosition)
    }
    /**
     * Gets the intersection of a ray-cast and the line
     * @param pose Pose of the ray-cast
     * @return Intersection point of ray-cast and line
     * @sample intersection
     * */
    fun intersection(pose: Pose2d) : Vector2? {
        return intersection(Vector2(pose.x.meters,pose.y.meters), pose.rotation.radians.radians)
    }
    /**
     * Gets the distance from the intersection of a ray-cast and the line
     * @param coordinate Origin of the ray-cast
     * @param rotation Rotation of the raycast
     * @return Distance from the intersection point of ray-cast and line
     * @sample distance
     * */
    fun distance(coordinate : Vector2, rotation : AngleUnit) : DistanceUnit? {
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
        return (Vector2(pose.x.meters,pose.y.meters) - intersectionPoint).magnitude
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