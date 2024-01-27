package frc.robot.subsytems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.utils.`M/s`
import frc.robot.Constants.DriveConstants
import frc.robot.commands.TeleOp
import frc.robot.subsystems.Odometry
import kotlin.math.absoluteValue

object Drivetrain : SubsystemBase() {
    private val       leftMain = CANSparkMax(DriveConstants.MotorLMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val  leftSecondary = CANSparkMax(DriveConstants.MotorLSubID,  CANSparkLowLevel.MotorType.kBrushless)
    private val      rightMain = CANSparkMax(DriveConstants.MotorRMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSecondary = CANSparkMax(DriveConstants.MotorRSubID,  CANSparkLowLevel.MotorType.kBrushless)

    val    leftEncoder: RelativeEncoder = leftMain.encoder
    val   rightEncoder: RelativeEncoder = rightMain.encoder

    private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val rightPid = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val FeedForward = SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA)

    private fun allMotors(code: CANSparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
        for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
            motor.apply(code)
        }
    }

    init {
        leftSecondary.follow(leftMain)
        rightSecondary.follow(rightMain)

        allMotors {
            restoreFactoryDefaults()
            setSmartCurrentLimit(DriveConstants.CurrentLimit) //Todo: there's a fancy version of this function that may be worth using
            //TODO: finish initialize spark maxes
        }

        drive.setDeadband(0.0)

        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
    }
    /** Runs the drive train
     * @param left Power for left motors [-1.0.. 1.0]. Forward is positive.
     * @param right Voltage for right motors [-1.0.. 1.0]. Forward is positive.
     * */
    fun tankDrive(left: Double, right: Double) {
        drive.tankDrive(left, right, false)
    }
    /** Sets drivetrain motor voltage directly
     * @param left Voltage for left motors
     * @param right Voltage for right motors
     * */
    fun rawDrive(left: Double, right: Double) {
        leftMain.setVoltage(left)
        rightMain.setVoltage(right)
    }
    /** Runs the rawDrive by using PID and FeedForward values to adjust voltage output and reach the desired speed.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: Double, right: Double) {
        leftPid.setpoint = left
        rightPid.setpoint = right

        val lPidCalculated = leftPid.calculate(leftEncoder.velocity)
        val rPidCalculated = rightPid.calculate(rightEncoder.velocity)

        val lFFCalculated = FeedForward.calculate(leftPid.setpoint)
        val rFFCalculated = FeedForward.calculate(rightPid.setpoint)

        rawDrive(lPidCalculated+lFFCalculated, rPidCalculated + rFFCalculated )
    }
    /** Runs the rawDrive by using PID and FeedForward values to adjust voltage output and reach the desired speed.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: `M/s`, right: `M/s`) { closedLoopDrive(left.value, right.value) }
    /** Runs the tankDrive using the curvature drive inverse kinematics from WPI, applies PID and FeedForward controls.
     * @param throttle The robots speed along the X-Axis [-1.0.. 1.0].
     * @param turn The normalized curvature [-1.0.. 1.0]. Counterclockwise is positive
     */
    fun rawCurvatureDrive(throttle:Double, turn:Double, allowTurnInPlace:Boolean = true) {
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(throttle,turn,true)
        tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    }
    /** Runs the tankDrive using the curvature drive inverse kinematics from WPI, applies PID and FeedForward controls.
     * @param throttle The robots speed along the X-Axis [-1.0.. 1.0].
     * @param turn The normalized curvature [-1.0.. 1.0]. Counterclockwise is positive
     */
    fun curvatureDrive(throttle:Double, turn:Double, maxSpeed: `M/s` = `M/s`(1.0), allowTurnInPlace:Boolean = true) {
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(throttle,turn,true)
        closedLoopDrive(wheelSpeeds.left * maxSpeed.value, wheelSpeeds.right * maxSpeed.value)
    }

}