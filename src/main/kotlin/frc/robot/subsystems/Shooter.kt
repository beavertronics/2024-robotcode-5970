package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.controls.toFeedForward
import frc.engine.controls.toPID
import frc.engine.utils.RPM
import frc.engine.utils.Rotations
import frc.engine.utils.RotationsPerSecond
import frc.engine.utils.Sugar.within
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.ShooterConstants as C

object Shooter : SubsystemBase() {
    private val leftFlywheel = CANSparkMax(C.LeftFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightFlywheel = CANSparkMax(C.RightFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)

    private val    leftEncoder: RelativeEncoder = leftFlywheel.encoder
    private val   rightEncoder: RelativeEncoder = rightFlywheel.encoder


    private val leftPid     = C.LEFT_PID_CONSTANTS.toPID()
    private val rightPid    = C.RIGHT_PID_CONSTANTS.toPID()
    private val leftFeedForward = C.LEFT_FF_CONSTANTS.toFeedForward()
    private val rightFeedForward = C.RIGHT_FF_CONSTANTS.toFeedForward()


    var targetSpeed = ShooterSpeeds()

    var testAmpSpeed = 0.RPM

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(C.CurrentLimit, leftFlywheel, rightFlywheel)

        // Invert the left flywheel
        leftFlywheel.inverted = true
        rightFlywheel.inverted = false



    }

    override fun periodic() {
        testAmpSpeed = SmartDashboard.getNumber("testAmpSpeedFrFr",0.0).RPM
        println(testAmpSpeed.rotationsPerMinute())
    }
    fun breakMotors(){
        leftFlywheel.idleMode = CANSparkBase.IdleMode.kBrake
        rightFlywheel.idleMode = CANSparkBase.IdleMode.kBrake
    }
    fun unBreakMotors(){
        leftFlywheel.idleMode = CANSparkBase.IdleMode.kCoast
        rightFlywheel.idleMode = CANSparkBase.IdleMode.kCoast
    }

    data class ShooterSpeeds(val leftSpeeds:Rotations = 0.RotationsPerSecond, val rightSpeeds:Rotations = 0.RotationsPerSecond)
    /**
     * Set the speed of the flywheels using closed loop control
     * @param leftSpeeds Desired speed of left the motor in RPM
     * @param rightSpeeds Desired speed of right the motor in RPM
     */
    fun setSpeed(leftSpeeds : Rotations, rightSpeeds : Rotations) {
        targetSpeed = ShooterSpeeds(leftSpeeds, rightSpeeds)
        leftPid.setpoint = leftSpeeds.rotationsPerSecond()
        rightPid.setpoint = rightSpeeds.rotationsPerSecond()
        //shooterMode = ShooterMode.CLOSED_LOOP
    }

    /** Calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(){
        val leftPidCalculated  = leftPid.calculate(leftEncoder.velocity.RPM.rotationsPerSecond())
        val rightPidCalculated = rightPid.calculate(rightEncoder.velocity.RPM.rotationsPerSecond())
        val leftFFCalculated   = leftFeedForward.calculate(targetSpeed.leftSpeeds.rotationsPerSecond())
        val rightFFCalculated  = rightFeedForward.calculate(targetSpeed.rightSpeeds.rotationsPerSecond())

        if(targetSpeed.leftSpeeds.value != 0.0) leftFlywheel.setVoltage(leftPidCalculated+leftFFCalculated)
        if(targetSpeed.rightSpeeds.value != 0.0) rightFlywheel.setVoltage(rightPidCalculated+rightFFCalculated)
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
        leftFlywheel.set(it.`in`(Units.Volts)/ RobotController.getBatteryVoltage())
        rightFlywheel.set(it.`in`(Units.Volts)/ RobotController.getBatteryVoltage())
    }

    private val m_appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.mutable(Units.Volts.of(0.0))
    private val m_distance: MutableMeasure<Angle> = MutableMeasure.mutable(Units.Rotations.of(0.0))
    private val m_velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.mutable(Units.RotationsPerSecond.of(0.0))

    /** Used for logging motor information, allowing information to be recorded for SysID */
    val logger: (SysIdRoutineLog) -> Unit =  {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("shoot-left")
            .voltage(
                m_appliedVoltage.mut_replace(
                    leftFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .angularPosition(m_distance.mut_replace(leftEncoder.position, Units.Rotations))
            .angularVelocity(
                m_velocity.mut_replace(leftEncoder.velocity, Units.RPM))
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("shoot-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                    rightFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .angularPosition(m_distance.mut_replace(rightEncoder.position, Units.Rotations))
            .angularVelocity(
                m_velocity.mut_replace(rightEncoder.velocity, Units.RPM))
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
    fun setSpeed(speed : Double) = setSpeed(speed.RPM, speed.RPM)

    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Desired speed of the motor
     */
    fun setSpeed(speed : Rotations) = setSpeed(speed, speed)
    /** First, sets the desired speed of the shooter
     * Then, calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(speed: Double){
        setSpeed(speed)
        runClosedLoop()
    }
    /** First, sets the desired speed of the shooter
     * Then, calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    fun runClosedLoop(speed: Rotations){
        setSpeed(speed)
        runClosedLoop()
    }

}