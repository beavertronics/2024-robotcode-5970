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
import frc.engine.utils.RPM
import frc.engine.utils.Sugar.within
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.ShooterConstants as C

object Shooter : SubsystemBase() {
    private val leftFlywheel = CANSparkMax(C.LeftFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightFlywheel = CANSparkMax(C.RightFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless)

    private val    leftEncoder: RelativeEncoder = leftFlywheel.encoder
    private val   rightEncoder: RelativeEncoder = rightFlywheel.encoder


    private val leftPid     = Controller.PID(C.KP, C.KD)
    private val rightPid    = Controller.PID(C.KP, C.KD)
    private val feedForward = SimpleMotorFeedforward(C.KS, C.KV, C.KA)

    var targetSpeed = ShooterSpeeds()

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(C.CurrentLimit, leftFlywheel, rightFlywheel)

        // Invert the left flywheel
        leftFlywheel.inverted = true
        rightFlywheel.inverted = false

        // Set the default command to idle
        defaultCommand = idle()
    }

    data class ShooterSpeeds(val leftSpeeds:RPM = 0.RPM, val rightSpeeds:RPM = 0.RPM)
    /**
     * Set the speed of the flywheels using closed loop control
     * @param leftSpeeds Desired speed of left the motor in RPM
     * @param rightSpeeds Desired speed of right the motor in RPM
     */
    fun setSpeed(leftSpeeds : Double, rightSpeeds: Double) = setSpeed(leftSpeeds.RPM, rightSpeeds.RPM)
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

    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Desired speed of the motor in RPM
     */
    fun setSpeed(speed : Double) = Shooter.setSpeed(speed, speed)

    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Desired speed of the motor in RPM
     */
    fun setSpeed(speed : RPM) = setSpeed(speed, speed)

    /** Calculates the PID & FeedForward, and sets the motors to the voltage to reach the desired speed */
    private fun runClosedLoop(){
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
    private fun stop(){
        leftFlywheel.set(0.0)
        rightFlywheel.set(0.0)
    }

    /** Run closed loop to reach the speed required to shoot into the speaker */
    fun doSpinupToSpeaker() : Command = doSpinup(C.SpeakerSpeed)

    /** Run closed loop to reach the speed required to shoot into the amp */
    fun doSpinupToAmp()     : Command = doSpinup(C.AmpSpeed)

    /** Run closed loop to reach speed
     * @param speed Desired speed in RPM */
    fun doSpinup(speed: RPM) : Command =
        this.run { runClosedLoop() }
        .beforeStarting ({ setSpeed(speed) })

    /** Run closed loop to reach speed, then end as soon as it reaches speed
     * @param speed Desired speed in RPM */
    fun doSpinupAndStop(speed: RPM) : Command =
        this.run { runClosedLoop() }
            .beforeStarting ({ setSpeed(speed) })
            .until { isAtSpeed }

    /**
     * Runs the shooter at voltage
     * @param voltages Voltages to run the robot on
     */
    fun doRunAtVoltage(voltages:Double = 0.0) : Command = this.run {
        leftFlywheel.setVoltage(voltages)
        rightFlywheel.setVoltage(voltages)
    }
    /**
     * Runs the shooter at voltage
     * @param voltages Voltage getter function, called each frame to set the shooter
     */
    fun doRunAtVoltage(voltages:() -> Double) : Command = this.run {
        leftFlywheel.setVoltage(voltages())
        rightFlywheel.setVoltage(voltages())
    }

    /** Stops the shooter */
    fun idle() : Command = this.run { stop() }

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


}