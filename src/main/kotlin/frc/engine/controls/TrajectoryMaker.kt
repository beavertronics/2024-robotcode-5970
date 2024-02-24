package frc.engine.controls
// File adapted from 2898's bpsrobotics engine
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import frc.engine.utils.Meters
import frc.engine.utils.MetersPerSecond
import frc.engine.utils.MetersPerSecondSquared

@Suppress("UNUSED")
class TrajectoryMaker(maxVel: MetersPerSecond, maxAccel: MetersPerSecondSquared) {

    private val config = TrajectoryConfig(
        maxVel.metersPerSecondValue(),
        maxAccel.metersPerSecondSquaredValue()
    )

    inner class TrajectoryBuilder internal constructor(private val startPose: Pose2d?, private val splinePoints: Array<Translation2d>, private val endPose: Pose2d?) {
        fun start(pose: Pose2d) = TrajectoryBuilder(pose, splinePoints, endPose)

        fun point(x: Meters, y: Meters) = TrajectoryBuilder(startPose, splinePoints + Translation2d(x.value, y.value), endPose)
        fun point(x: Double, y: Double) = TrajectoryBuilder(startPose, splinePoints + Translation2d(x, y), endPose)
        fun point(point: Translation2d) = TrajectoryBuilder(startPose, splinePoints + point, endPose)
        fun points(vararg points: Translation2d) = TrajectoryBuilder(startPose, splinePoints + points, endPose)
        fun points(points: Iterable<Translation2d>) = TrajectoryBuilder(startPose, splinePoints + points.toList(), endPose)

        fun end(pose: Pose2d) = TrajectoryBuilder(startPose, splinePoints, pose)

        fun build(): Trajectory {
            return TrajectoryGenerator.generateTrajectory(startPose, splinePoints.toList(), endPose, config)
        }
    }

    fun builder() = TrajectoryBuilder(null, emptyArray(), null)
}