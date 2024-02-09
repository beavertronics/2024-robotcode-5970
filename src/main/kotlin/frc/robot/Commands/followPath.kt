package frc.robot.Commands

import com.pathplanner.lib.commands.FollowPathRamsete
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.frcUtils.getAlliance
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Odometry

fun followPathCommand(pathName: String) : Command {
    val path = PathPlannerPath.fromPathFile(pathName);
    val getAllianceLambda : () -> Boolean =
        {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            getAlliance()

        }
    return FollowPathRamsete(
        path,
        Odometry.getPose, // Robot pose supplier
        Odometry.getChassesSpeeds, // Current ChassisSpeeds supplier
        Drivetrain.consumeDrive, // Method that will drive the robot given ChassisSpeeds
        ReplanningConfig(), // Default path replanning config. See the API for the options here
        getAllianceLambda,
        Drivetrain // Reference to this subsystem to set requirements
    )
}