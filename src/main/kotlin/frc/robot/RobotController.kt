package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj2.command.CommandScheduler

import frc.robot.commands.TeleOp

/* Main code for controlling the robot. Mainly just links everything together.

    The hardware of the robot (what motor controllers, etc) is defined in RobotHardware.kt

    Driver control is defined in TeleOp.kt

*/

object RobotController : TimedRobot() {
    val bot = RobotHardware
    
    override fun robotInit() {
        //Initialize the robot!
        CameraServer.startAutomaticCapture() //TODO: Can we offload camera streaming to a Raspberry Pi?

        SmartDashboard.putData(RobotHardware.Drive)
    }
    override fun robotPeriodic() {
        //Runs while the robot is on, regarless of whether it is enabled.
        // (use for telemetry, command scheduler)
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {}
    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        selectedAuto.cancel() //Stop Auto before running teleop!
        TeleOp.schedule()
    }
    override fun teleopPeriodic() {}

    override fun simulationInit() {} //Runs only in simulation mode (other functions run regardless of whether robot is simulated or not)

    override fun disabledInit() {
        //Runs as soon as the robot is disabled, use to deactivate motors safely, etc
    }
    override fun disabledPeriodic() {
        //Runs only while robot is disabled- Use to hold motors in position for safety reasons.
        // Try to avoid putting code here- often unsafe.
    }

    override fun testInit() {}
    override fun testPeriodic() {}
}