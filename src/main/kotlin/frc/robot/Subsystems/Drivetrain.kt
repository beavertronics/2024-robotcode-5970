package frc.robot.subsytems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.utils.`M/s`
import frc.robot.RobotHardware.Drive
import frc.robot.Constants.DrivetrainConstants

object Drivetrain : SubsystemBase() {
    private val leftPid = Controller.PID(DrivetrainConstants.KP, DrivetrainConstants.KD)
    private val rightPid = Controller.PID(DrivetrainConstants.KP, DrivetrainConstants.KD)
    private val FeedForward = SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV, DrivetrainConstants.KA)
    fun rawDrive(left: Double, right: Double) {
        Drive.drive.tankDrive(left, right, false)
    }

    fun closedLoopDrive(left: `M/s`, right: `M/s`) {
        leftPid.setpoint = left.value
        rightPid.setpoint = right.value
    }
    
}