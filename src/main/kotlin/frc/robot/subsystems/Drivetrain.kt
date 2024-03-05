package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.controls.Ramsete
import frc.engine.controls.TrajectoryMaker
import frc.engine.utils.`M/s`
import frc.engine.utils.initMotorControllers
import frc.engine.utils.*
import frc.robot.Constants.DriveConstants
//import frc.robot.subsystems.Odometry.chassisSpeeds


object Drivetrain : SubsystemBase() {
    private val       leftMain = CANSparkMax(DriveConstants.MotorLMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val  leftSecondary = CANSparkMax(DriveConstants.MotorLSubID,  CANSparkLowLevel.MotorType.kBrushless)
    private val      rightMain = CANSparkMax(DriveConstants.MotorRMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSecondary = CANSparkMax(DriveConstants.MotorRSubID,  CANSparkLowLevel.MotorType.kBrushless)

    val    leftEncoder: RelativeEncoder = leftMain.encoder
    val   rightEncoder: RelativeEncoder = rightMain.encoder

    private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid  = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val rightPid = Controller.PID(DriveConstants.KP, DriveConstants.KD)
    private val leftFeedForward  = SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA)
    private val rightFeedForward = SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA)


    val trajectoryMaker = TrajectoryMaker(DriveConstants.MaxVelocity, DriveConstants.MaxAcceleration)
    var trajectory : Trajectory? = null
    private var trajectoryStartTime = 0.seconds

    private val ramsete: Ramsete = Ramsete(
        DriveConstants.TrackWidth.toMeters(),
        Odometry,
        leftPid,
        rightPid,
        leftFeedForward,
        rightFeedForward,
        DriveConstants.DRIVETRAIN_RAMSETE_B,
        DriveConstants.DRIVETRAIN_RAMSETE_Z
    )

    private fun allMotors(code: CANSparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
        for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
            motor.apply(code)
        }
    }

    init {
        initMotorControllers(DriveConstants.CurrentLimit, leftMain, rightMain, leftSecondary, rightSecondary)

        leftSecondary.follow(leftMain)
        rightSecondary.follow(rightMain)

        drive.setDeadband(0.0)

        leftMain.inverted = true
        leftSecondary.inverted = true
    }
    /** Drive by setting left and right power (-1 to 1).
     * @param left Power for left motors [-1.0.. 1.0]. Forward is positive.
     * @param right Voltage for right motors [-1.0.. 1.0]. Forward is positive.
     * */
    fun tankDrive(left: Double, right: Double) {
        drive.tankDrive(left, right, false)
    }
    /** Drive by setting left and right voltage (-12v to 12v)
     * @param left Voltage for left motors
     * @param right Voltage for right motors
     * */
    fun rawDrive(left: Double, right: Double) {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftMain.setVoltage(left)
        rightMain.setVoltage(right)
        drive.feed()
    }
    fun rawDrive(voltages: Ramsete.WheelVoltages) = rawDrive(voltages.left.value, voltages.right.value)

    fun stop(){
        rawDrive(0.0,0.0)
    }
    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: Double, right: Double) {
        leftPid.setpoint = left
        rightPid.setpoint = right

        val lPidCalculated = leftPid.calculate(leftEncoder.velocity)
        val rPidCalculated = rightPid.calculate(rightEncoder.velocity)

        val lFFCalculated = leftFeedForward.calculate(leftPid.setpoint)
        val rFFCalculated = rightFeedForward.calculate(rightPid.setpoint)

        rawDrive(lPidCalculated+lFFCalculated, rPidCalculated + rFFCalculated )
    }

    fun followPath(traj : Trajectory? = null){
        if(traj != null) {
            trajectoryStartTime = Timer.getFPGATimestamp().seconds
            trajectory = traj
        }
        if (trajectory == null) {
            rawDrive(0.0, 0.0)
            return
        }
        Odometry.field.getObject("trajectory").setTrajectory(traj)

        rawDrive(
            ramsete.voltages(
                trajectory!!,
                Timer.getFPGATimestamp().seconds - trajectoryStartTime,
                Odometry.vels
            )
        )
    }

    private fun rawDrive(left: Double) {

    }

    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: `M/s`, right: `M/s`) { closedLoopDrive(left.value, right.value) }
    fun closedLoopDrive(speeds: ChassisSpeeds){ //Todo: speeds is passed directly from odometry
        val kinematics = DifferentialDriveKinematics(DriveConstants.TrackWidth.value)
        val wheelSpeeds: DifferentialDriveWheelSpeeds = kinematics.toWheelSpeeds(speeds)
        closedLoopDrive(wheelSpeeds.leftMetersPerSecond,wheelSpeeds.rightMetersPerSecond)
     }
    val consumeDrive: (ChassisSpeeds) -> Unit = {
        closedLoopDrive(it)
    }

    /** Drive by setting left and right voltage (-12v to 12v)
     * @param volts Voltage for motors motors
     * */
    val rawDrive: (Measure<Voltage>) -> Unit =  {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftMain.setVoltage(it.`in`(Units.Volts))
        rightMain.setVoltage(it.`in`(Units.Volts))
    }

    private val m_appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.mutable(Units.Volts.of(0.0))
    private val m_distance: MutableMeasure<Distance> = MutableMeasure.mutable(Units.Meters.of(0.0))
    private val m_velocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.mutable(Units.MetersPerSecond.of(0.0))

    val logger: (SysIdRoutineLog) -> Unit =  {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("drive-left")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                leftMain.get() * RobotController.getBatteryVoltage(), Units.Volts
                        ))
                .linearPosition(m_distance.mut_replace(leftEncoder.position, Units.Meters))
                .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.velocity, Units.MetersPerSecond));
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("drive-right")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                rightMain.get() * RobotController.getBatteryVoltage(), Units.Volts
                        ))
                .linearPosition(m_distance.mut_replace(rightEncoder.position, Units.Meters))
                .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.velocity, Units.MetersPerSecond));
    }

}