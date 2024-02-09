package frc.engine.odometry
// File adapted from 2898's bpsrobotics engine
import frc.engine.utils.Degrees
import frc.engine.utils.Meters
import frc.engine.utils.deg
import frc.engine.utils.m
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import java.util.function.Supplier

/** Provides an orientation and position for the robot */
interface PoseProvider : Sendable {
    /** Provides the pose as a WPIlib Pose2d */
    val pose: Pose2d

    /** Updates the [pose] variable */
    fun update()

    fun reset(x: Meters, y: Meters, theta: Degrees)
    fun reset(pose2d: Pose2d) {
        reset(pose2d.x.m, pose2d.y.m, pose2d.rotation.degrees.deg)
    }
    fun getPose() : Pose2d{
        return pose
    }
    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}