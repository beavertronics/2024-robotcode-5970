package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
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
        CLOSED, OPEN
    }
    var shooterMode = ShooterMode.OPEN

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
        shooterMode = ShooterMode.OPEN

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
    fun stop() = setSpeed(0.RPM,0.RPM)
    fun shootSpeaker(distToSpeaker: Double){
        setSpeed(C.SpeakerPoly.calculate(distToSpeaker))
    }
    fun setSpeedRaw(speed: Double) {
        shooterMode = ShooterMode.CLOSED
        rawShooterSpeed = speed
    }


    override fun periodic() {
        when {
            shooterMode == ShooterMode.OPEN -> {
                val leftpid = leftPid.calculate(leftEncoder.velocity)
                val rightpid = leftPid.calculate(rightEncoder.velocity)
                val leftFF = feedForward.calculate(targetSpeed.leftSpeeds.rotationsPerMinute())
                val rightFF = feedForward.calculate(targetSpeed.rightSpeeds.rotationsPerMinute())

                leftFlywheel.setVoltage(leftpid+leftFF)
                rightFlywheel.setVoltage(rightpid+rightFF)
            }
            shooterMode == ShooterMode.CLOSED -> {
                leftFlywheel.setVoltage(rawShooterSpeed)
                rightFlywheel.setVoltage(rawShooterSpeed)
            }

        }

    }

}