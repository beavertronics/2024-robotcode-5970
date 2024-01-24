package frc.robot.subsytems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotHardware.Drive
import frc.robot.Constants.DrivetrainConstants

object Drivetrain : SubsystemBase() {
    private val leftPid = Controller.PID(DrivetrainConstants.KP, DrivetrainConstants.KD)
    private val rightPid = Controller.PID(DrivetrainConstants.KP, DrivetrainConstants.KD)

    fun rawDrive(left: Double, right: Double) {
        Drive.drive.tankDrive(left, right, false)
    }

    fun test(left: Double, right: Double) {
        leftPid.setpoint = left
        rightPid.setpoint = right
    }
    
}