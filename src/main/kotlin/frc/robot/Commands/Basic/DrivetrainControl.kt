package frc.robot.Commands.Basic

import com.pathplanner.lib.commands.FollowPathRamsete
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.frcUtils
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Odometry

object DrivetrainControl {
    fun followPathCommand(pathName: String, resetOdometry : Boolean = false) : Command {
        val path = PathPlannerPath.fromPathFile("../../../../pathplanner/deploy/pathplanner/paths/${pathName}.path");
        val getAllianceLambda : () -> Boolean =
            {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                frcUtils.getAlliance()

            }
        val pathFollow: FollowPathRamsete = FollowPathRamsete(
            path,
            Odometry.getPose, // Robot pose supplier
            Odometry.getChassesSpeeds, // Current ChassisSpeeds supplier
            Drivetrain.consumeDrive, // Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(), // Default path replanning config. See the API for the options here
            getAllianceLambda,
            Drivetrain // Reference to this subsystem to set requirements
        )
        if(resetOdometry) pathFollow.beforeStarting({ Odometry.reset(path.startingDifferentialPose)})
        return pathFollow
    }
    class runDrivetrain(private val voltage: Double,
                        private val time: Double) : Command() {
        private val timer = Timer()
        override fun initialize() = timer.restart()
        override fun execute() = Drivetrain.voltageDrive(voltage, voltage)
        override fun end(interrupted: Boolean) = Drivetrain.stop()
        override fun isFinished(): Boolean = timer.hasElapsed(time)

    }
}