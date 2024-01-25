package frc.robot

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel.MotorType

import com.kauailabs.navx.frc.AHRS as NAVX

import frc.robot.Constants.DriveConstants as D

/* Object that represents the robot's hardware, organized into subsystems.

Code for *controlling* this hardware should be located elsewhere (ie in RobotController).
This should just include the low-level components, ie motor controllers and sensors.

PID control objects can go here also, and wrappers, but try to keep any real logic in TeleOp.kt or one of the auto files

*/

object RobotHardware {

    object Drive : SubsystemBase() {

        private val       leftMain = CANSparkMax(D.MotorRMainID,MotorType.kBrushless)
        private val  leftSecondary = CANSparkMax(D.MotorRSubID,MotorType.kBrushless)
        private val      rightMain = CANSparkMax(D.MotorRMainID,MotorType.kBrushless)
        private val rightSecondary = CANSparkMax(D.MotorRSubID,MotorType.kBrushless)

        private val drive = DifferentialDrive(leftMain, rightMain)

        private val    leftEncoder = leftMain.getEncoder()
        private val   rightEncoder = rightMain.getEncoder()

        val leftPID  = PIDController(D.KP, D.KI, D.KD) //TODO: Maybe switch out for spark max integrated PIDs? (easy as controller.getPIDController())
        val rightPID = PIDController(D.KP, D.KI, D.KD)
        val feedforward = SimpleMotorFeedforward(D.KS, D.KV, D.KA) //Feedforward has no internal state, so you only need one for both

        val imu =  NAVX()

        // WRAPPERS
        fun updateSetpoints(left: Double, right: Double) {
            leftPID.setpoint = left;
            rightPID.setpoint = right;
        }

        fun allMotors(code: CANSparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
            for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
                motor.apply(code)
            }
        }
        //For non-tankdrive, use corrosponding IK method of DifferentialDrive instead and feed the wheelSpeeds into tankDrive.
        fun tankDrive(leftSpeed : Double, rightSpeed : Double) = drive.tankDrive(leftSpeed, rightSpeed)
        fun tankDrive(speeds : DifferentialDrive.WheelSpeeds) = drive.tankDrive(speeds.left, speeds.right)

        init {
            leftSecondary.follow(leftMain)
            rightSecondary.follow(rightMain)

            allMotors {
                restoreFactoryDefaults()
                //TODO: initialize spark maxes
            }
            /* TODO: set motor inversion correctly
            leftMain.inverted = false
            leftSecondary.inverted = false
            rightMain.inverted = true
            rightSecondary.inverted = true
            */
        }
    }
    /*TODO: Intake Hardware
    class Intake : SubsystemBase() {
        t o d o
    }
    */
}