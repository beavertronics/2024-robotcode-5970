package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.controls.toFeedForward
import frc.engine.controls.toPID
import frc.engine.utils.RPM
import frc.engine.utils.Sugar.within
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.ShooterConstants as C

object Shooter : SubsystemBase() {
    private val leftFlywheel = CANSparkMax(C.LeftFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightFlywheel = CANSparkMax(C.RightFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)

    private val    leftEncoder: RelativeEncoder = leftFlywheel.encoder
    private val   rightEncoder: RelativeEncoder = rightFlywheel.encoder


    private val leftPid     = C.PID_CONSTANTS.toPID()
    private val rightPid    = C.PID_CONSTANTS.toPID()
    private val feedForward = C.FF_CONSTANTS.toFeedForward()

    var targetSpeed = ShooterSpeeds()

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(C.CurrentLimit, leftFlywheel, rightFlywheel)

        // Invert the left flywheel
        leftFlywheel.inverted = true
        rightFlywheel.inverted = false
    }

    data class ShooterSpeeds(val leftSpeeds:RPM = 0.RPM, val rightSpeeds:RPM = 0.RPM)
    /**
     * Set the speed of the flywheels using closed loop control
     * @param leftSpeeds Desired speed of left the motor in RPM
     * @param rightSpeeds Desired speed of right the motor in RPM
     */
    fun setSpeed(leftSpeeds : RPM, rightSpeeds : RPM) {
        //targetSpeed = ShooterSpeeds(leftSpeeds, rightSpeeds)
        leftPid.setpoint = leftSpeeds.rotationsPerMinute()
        rightPid.setpoint = rightSpeeds.rotationsPerMinute()
        //shooterMode = ShooterMode.CLOSED_LOOP
    }

    /** Calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(){
        val leftPidCalculated  = leftPid.calculate(leftEncoder.velocity)
        val rightPidCalculated = leftPid.calculate(rightEncoder.velocity)
        val leftFFCalculated   = feedForward.calculate(targetSpeed.leftSpeeds.rotationsPerMinute())
        val rightFFCalculated  = feedForward.calculate(targetSpeed.rightSpeeds.rotationsPerMinute())

        leftFlywheel.setVoltage(leftPidCalculated+leftFFCalculated)
        rightFlywheel.setVoltage(rightPidCalculated+rightFFCalculated)
    }

    /** Runs the flywheels at percentShooterSpeed
     * @param percentShooterSpeed The percent at which to run the flywheel */
    fun runOpenLoop(percentShooterSpeed : Double){
        leftFlywheel.set(percentShooterSpeed)
        rightFlywheel.set(percentShooterSpeed)
    }
    /** Runs the flywheels at 0% stopping them from continuing to run */
    fun stop(){
        leftFlywheel.set(0.0)
        rightFlywheel.set(0.0)
    }
    /** Returns true if the encoder velocity is equal to the desired speed */
    val isAtSpeed get() = (leftEncoder.velocity.within(10.0, targetSpeed.leftSpeeds.value) &&
            rightEncoder.velocity.within(10.0, targetSpeed.rightSpeeds.value))





    /** Charactarization only DON'T USE */
    val rawDrive: (Measure<Voltage>) -> Unit =  {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftFlywheel.setVoltage(it.`in`(Units.Volts))
        rightFlywheel.setVoltage(it.`in`(Units.Volts))
    }

    private val m_appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.mutable(Units.Volts.of(0.0))
    private val m_distance: MutableMeasure<Distance> = MutableMeasure.mutable(Units.Meters.of(0.0))
    private val m_velocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.mutable(Units.MetersPerSecond.of(0.0))

    /** Used for logging motor information, allowing information to be recorded for SysID */
    val logger: (SysIdRoutineLog) -> Unit =  {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("shoot-left")
            .voltage(
                m_appliedVoltage.mut_replace(
                    leftFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .linearPosition(m_distance.mut_replace(leftEncoder.position, Units.Meters))
            .linearVelocity(
                m_velocity.mut_replace(leftEncoder.velocity, Units.MetersPerSecond))
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("shoot-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                    rightFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .linearPosition(m_distance.mut_replace(rightEncoder.position, Units.Meters))
            .linearVelocity(
                m_velocity.mut_replace(rightEncoder.velocity, Units.MetersPerSecond))
    }
    /**
     * Set the speed of the flywheels using closed loop control
     * @param leftSpeeds Desired speed of left the motor in RPM
     * @param rightSpeeds Desired speed of right the motor in RPM
     */
    fun setSpeed(leftSpeeds : Double, rightSpeeds: Double) = setSpeed(leftSpeeds.RPM, rightSpeeds.RPM)
    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Desired speed of the motor in RPM
     */
    fun setSpeed(speed : Double) = setSpeed(speed, speed)

    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Desired speed of the motor in RPM
     */
    fun setSpeed(speed : RPM) = setSpeed(speed, speed)
    /** First, sets the desired speed of the shooter
     * Then, calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(speed: Double){
        setSpeed(speed)
        runClosedLoop()
    }
    /** First, sets the desired speed of the shooter
     * Then, calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(speed: RPM){
        setSpeed(speed)
        runClosedLoop()
    }

}