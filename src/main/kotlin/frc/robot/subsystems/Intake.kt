package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    private val limitSwitch = DigitalInput(C.limitSwitchChannel)
    private val feedingTimer = Timer()
    enum class IntakeState {
        INTAKING, PULLBACK, LOADED, FEEDING, IDLE, OUTTAKING
    }

    var intakeState = IntakeState.IDLE

    private val TopMotor = CANSparkMax(C.TopMotorID, CANSparkLowLevel.MotorType.kBrushed)
    private val bottomMotor = TalonSRX(C.BottomMotorID)

    init {
        TopMotor.setSmartCurrentLimit(C.CurrentLimit)
        TopMotor.restoreFactoryDefaults()
        bottomMotor.configContinuousCurrentLimit(C.CurrentLimit)
        bottomMotor.configFactoryDefault()
        bottomMotor.inverted = true
        TopMotor.inverted = false

    }

    /** Runs the intake motor at the give voltage
     * @param voltage The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntakeRaw(voltage:Double) {
        bottomMotor.set(ControlMode.PercentOutput, voltage/12)
        TopMotor.set(voltage/12)
    }
    /** Runs the intake motor at 0V, stopping it
     */
    fun stop() {
        bottomMotor.set(ControlMode.PercentOutput, 0.0)
        TopMotor.set(0.0)
    }
    fun runIntake(speed:Double) {
        bottomMotor.set(ControlMode.PercentOutput, speed)
        TopMotor.set(speed)

    }
    fun intakeNote() {
        if (intakeState == IntakeState.IDLE || intakeState == IntakeState.OUTTAKING) {
            intakeState = IntakeState.INTAKING
        }
    }
    fun stopIntakingNote() {

        if (intakeState == IntakeState.INTAKING || intakeState == IntakeState.OUTTAKING || intakeState == IntakeState.LOADED) {
            intakeState = IntakeState.IDLE
        }
    }
    fun startFeeding() {
        if(intakeState != IntakeState.FEEDING)
        {
            feedingTimer.reset()
            feedingTimer.start()
        }
        intakeState = IntakeState.FEEDING
    }
    fun outtake() {
        intakeState = IntakeState.OUTTAKING
    }

    override fun periodic() {
        when (intakeState) {
            IntakeState.INTAKING -> {
                runIntake(C.intakeSpeed)
                if (!limitSwitch.get()) {
                    intakeState = IntakeState.PULLBACK
                }
            }
            IntakeState.PULLBACK -> {
                runIntake(-1 * C.pullbackSpeed)
                if (limitSwitch.get()) {
                    intakeState = IntakeState.LOADED
                }

            }
            IntakeState.FEEDING -> {
                runIntake(C.feedingSpeed)
                if (feedingTimer.hasElapsed(C.feedingTime)) {
                    feedingTimer.stop()
                    stop()
                    intakeState = IntakeState.IDLE
                }
            }
            IntakeState.OUTTAKING -> runIntake(-C.outtakeSpeed)
            else -> stop()
        }
    }

}