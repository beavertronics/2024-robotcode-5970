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
import frc.engine.controls.*
import frc.engine.utils.`M/s`
import frc.engine.utils.initMotorControllers
import frc.engine.utils.*
import frc.robot.Constants.DriveConstants
//import frc.robot.subsystems.Odometry.chassisSpeeds


object Drivetrain : SubsystemBase() {
    private val       leftMain = CANSparkMax(DriveConstants.LeftMotorMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val  leftSecondary = CANSparkMax(DriveConstants.LeftMotorSubID,  CANSparkLowLevel.MotorType.kBrushless)
    private val      rightMain = CANSparkMax(DriveConstants.RightMotorMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSecondary = CANSparkMax(DriveConstants.RightMotorSubID,  CANSparkLowLevel.MotorType.kBrushless)

    val    leftEncoder: RelativeEncoder = leftMain.encoder
    val   rightEncoder: RelativeEncoder = rightMain.encoder

    private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid  = DriveConstants.PID_CONSTANTS.toPID()
    private val rightPid = DriveConstants.PID_CONSTANTS.toPID()
    private val leftFeedForward  = DriveConstants.FF_CONSTANTS.toFeedForward()
    private val rightFeedForward = DriveConstants.FF_CONSTANTS.toFeedForward()


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

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(DriveConstants.CurrentLimit, leftMain, rightMain, leftSecondary, rightSecondary)

        // Set secondary motors to follow the primary ones
        leftSecondary.follow(leftMain)
        rightSecondary.follow(rightMain)

        // Set the dead band to 0, as this is handled elsewhere in the code.
        drive.setDeadband(0.0)

        // Invert the left motors
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
    fun voltageDrive(left: Double, right: Double) {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not necessary?
        leftMain.setVoltage(left)
        rightMain.setVoltage(right)
        drive.feed()
    }

    /**
     * Applies Ramsete wheel voltages to the motors
     */
    fun voltageDrive(voltages: Ramsete.WheelVoltages) = voltageDrive(voltages.left.value, voltages.right.value)
    /** Drive by setting left and right voltage (-12v to 12v)
     * @param volts Voltage for motors motors
     * */
    val voltageDrive: (Measure<Voltage>) -> Unit =  {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not necessary?
        leftMain.setVoltage(it.`in`(Units.Volts))
        rightMain.setVoltage(it.`in`(Units.Volts))
    }

    /**
     * Sets the motor voltage to 0 to prevent motors from running
     */
    fun stop(){
        voltageDrive(0.0,0.0)
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

        voltageDrive(lPidCalculated+lFFCalculated, rPidCalculated + rFFCalculated )
    }

    /**
     * Sets the path for the drivetrain to follow, if Trajectory is null, it will continue to follow the last path it was given
     * Required to be updated each frame in order to work properly
     * @param traj Trajectory to follow (null to continue last path)
     */
    fun followPath(traj : Trajectory? = null){
        if(traj != null) {
            trajectoryStartTime = Timer.getFPGATimestamp().seconds
            trajectory = traj
        }
        if (trajectory == null) {
            voltageDrive(0.0, 0.0)
            return
        }
        Odometry.field.getObject("trajectory").setTrajectory(traj)

        voltageDrive(
            ramsete.voltages(
                trajectory!!,
                Timer.getFPGATimestamp().seconds - trajectoryStartTime,
                Odometry.vels
            )
        )
    }
    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: `M/s`, right: `M/s`) { closedLoopDrive(left.value, right.value) }
    /** Drive by getting left and right speed, in M/s, from Chassis speeds using PID and FeedForward to correct for errors.
     * @param speeds Desired ChassisSpeeds
     */
    fun closedLoopDrive(speeds: ChassisSpeeds){ //Todo: speeds is passed directly from odometry
        val kinematics = DifferentialDriveKinematics(DriveConstants.TrackWidth.value)
        val wheelSpeeds: DifferentialDriveWheelSpeeds = kinematics.toWheelSpeeds(speeds)
        closedLoopDrive(wheelSpeeds.leftMetersPerSecond,wheelSpeeds.rightMetersPerSecond)
     }
    /** Lambda for driving by getting left and right speed, in M/s, from Chassis speeds using PID and FeedForward to correct for errors. */
    val consumeDrive: (ChassisSpeeds) -> Unit = {
        closedLoopDrive(it)
    }

    private val m_appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.mutable(Units.Volts.of(0.0))
    private val m_distance: MutableMeasure<Distance> = MutableMeasure.mutable(Units.Meters.of(0.0))
    private val m_velocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.mutable(Units.MetersPerSecond.of(0.0))

    /**
     * Used for logging motor information, allowing information to be recorded for SysID
     */

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
                        m_velocity.mut_replace(leftEncoder.velocity, Units.MetersPerSecond))
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("drive-right")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                rightMain.get() * RobotController.getBatteryVoltage(), Units.Volts
                        ))
                .linearPosition(m_distance.mut_replace(rightEncoder.position, Units.Meters))
                .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.velocity, Units.MetersPerSecond))
    }

}