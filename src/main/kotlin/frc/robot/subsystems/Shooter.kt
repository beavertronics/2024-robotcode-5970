package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.controls.Controller
import frc.engine.utils.RPM
import frc.robot.Constants.ShooterConstants as C

object Shooter : SubsystemBase() {
    private val leftFlywheel = CANSparkMax(C.leftFlywheel, CANSparkLowLevel.MotorType.kBrushless)
    private val rightFlywheel = CANSparkMax(C.rightFlywheel, CANSparkLowLevel.MotorType.kBrushless)

    private val    leftEncoder: RelativeEncoder = leftFlywheel.encoder
    private val   rightEncoder: RelativeEncoder = rightFlywheel.encoder


    private val leftPid     = Controller.PID(C.KP, C.KD)
    private val rightPid    = Controller.PID(C.KP, C.KD)
    private val feedForward = SimpleMotorFeedforward(C.KS, C.KV, C.KA)

    private var rawShooterSpeed = 0.0;

    enum class ShooterMode {
        CLOSED_LOOP, OPEN_LOOP, STOP
    }
    var shooterMode = ShooterMode.OPEN_LOOP

    var targetSpeed = ShooterSpeeds()

    init {

        leftFlywheel.restoreFactoryDefaults()
        leftFlywheel.setSmartCurrentLimit(C.CurrentLimit) //Todo: there's a fancy version of this function that may be worth using
            //TODO: finish initialize spark maxes


        // TODO: set motor inversion correctly
        leftFlywheel.inverted = false
        rightFlywheel.inverted = true
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
        targetSpeed = ShooterSpeeds(leftSpeeds, rightSpeeds)
        leftPid.setpoint = leftSpeeds.rotationsPerMinute()
        rightPid.setpoint = rightSpeeds.rotationsPerMinute()
        shooterMode = ShooterMode.CLOSED_LOOP

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
    fun stop() {
        shooterMode=ShooterMode.STOP

    }

    fun shootSpeaker(distToSpeaker: Double){
        setSpeed(C.SpeakerPoly.calculate(distToSpeaker))
    }
    fun setSpeedRaw(speed: Double) {
        shooterMode = ShooterMode.OPEN_LOOP
        rawShooterSpeed = speed
    }


    /**
     * Charactarization only DON'T USE
     */
    val rawDrive: (Measure<Voltage>) -> Unit =  {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftFlywheel.setVoltage(it.`in`(Units.Volts))
        rightFlywheel.setVoltage(it.`in`(Units.Volts))
    }


    override fun periodic() {
        when(shooterMode) {
            ShooterMode.CLOSED_LOOP -> {
                val leftpid = leftPid.calculate(leftEncoder.velocity)
                val rightpid = leftPid.calculate(rightEncoder.velocity)
                val leftFF = feedForward.calculate(targetSpeed.leftSpeeds.rotationsPerMinute())
                val rightFF = feedForward.calculate(targetSpeed.rightSpeeds.rotationsPerMinute())

                leftFlywheel.setVoltage(leftpid+leftFF)
                rightFlywheel.setVoltage(rightpid+rightFF)
            }
            ShooterMode.OPEN_LOOP -> {
                leftFlywheel.set(rawShooterSpeed)
                rightFlywheel.set(rawShooterSpeed)
            }
            else -> {
                leftFlywheel.set(0.0)
                rightFlywheel.set(0.0)
            }
        }

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
                    leftFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .linearPosition(m_distance.mut_replace(Drivetrain.leftEncoder.position, Units.Meters))
            .linearVelocity(
                m_velocity.mut_replace(Drivetrain.leftEncoder.velocity, Units.MetersPerSecond));
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        it.motor("drive-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                    rightFlywheel.get() * RobotController.getBatteryVoltage(), Units.Volts
                ))
            .linearPosition(m_distance.mut_replace(Drivetrain.rightEncoder.position, Units.Meters))
            .linearVelocity(
                m_velocity.mut_replace(Drivetrain.rightEncoder.velocity, Units.MetersPerSecond));
    }


}