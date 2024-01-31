package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.Victor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.utils.`M/s`
import frc.robot.Constants.DriveConstants

object Drivetrain : SubsystemBase() {
    private val       leftMain = VictorSPX(DriveConstants.MotorLMainID)
    private val  leftSecondary = VictorSPX(DriveConstants.MotorLSubID)
    private val      rightMain = VictorSPX(DriveConstants.MotorRMainID)
    private val rightSecondary = VictorSPX(DriveConstants.MotorRSubID)

    /*val    leftEncoder: RelativeEncoder = rightMain.encoder

    val   rightEncoder: RelativeEncoder = rightMain.encoder*/

    //private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val rightPid = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val FeedForward = SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA)

    private fun allMotors(code: VictorSPX.() -> Unit) { //Run a piece of code for each drive motor controller.
        for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
            motor.apply(code)
        }
    }

    init {
        leftSecondary.follow(leftMain)
        rightSecondary.follow(rightMain)

        allMotors {
            configFactoryDefault()
            //setSmartCurrentLimit(DriveConstants.CurrentLimit) //Todo: there's a fancy version of this function that may be worth using
            //TODO: finish initialize spark maxes
        }

        //drive.setDeadband(0.0)
        leftMain.inverted = true
        leftSecondary.inverted = true

        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
    }
    fun percentDrive(left_speeds: Double, right_speed: Double){
        leftMain.set(ControlMode.PercentOutput, left_speeds)
        rightMain.set(ControlMode.PercentOutput, right_speed)
    }
    fun percentCurvatureDrive(throttle:Double, turn:Double, allowTurnInPlace:Boolean = true) {
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(throttle,turn,true)
        percentDrive(wheelSpeeds.left, wheelSpeeds.right)
    }


}