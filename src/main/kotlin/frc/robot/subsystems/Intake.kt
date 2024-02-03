package frc.robot.subsystems

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

    private val DriverMotor = CANSparkMax(Constants.DriveConstants.MotorLMainID, CANSparkLowLevel.MotorType.kBrushless)
    init {
        DriverMotor.setSmartCurrentLimit(C.CurrentLimit)
        DriverMotor.restoreFactoryDefaults()
    }

    /** Runs the intake motor at the give voltage
     * @param voltage The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntakeRaw(voltage:Double) {
        DriverMotor.setVoltage(voltage)
    }
    /** Runs the intake motor at 0V, stopping it
     */
    fun stop() { DriverMotor.setVoltage(0.0) }
    fun runIntake(speed:Double) {
        DriverMotor.set(speed)
    }
    fun intakeNote() {
        if (intakeState == IntakeState.IDLE) {
            intakeState = IntakeState.INTAKING
        }
    }
    fun stopIntakingNote() {
        if (intakeState == IntakeState.INTAKING) {
            intakeState = IntakeState.IDLE
        }
    }
    fun startFeeding() {
        intakeState = IntakeState.FEEDING
        feedingTimer.reset()
        feedingTimer.start()
    }
    fun outtake() {
        intakeState = IntakeState.OUTTAKING
    }

    override fun periodic() {
        when (intakeState) {
            IntakeState.INTAKING -> {
                runIntake(C.intakeSpeed)
                if (limitSwitch.get()) {
                    intakeState = IntakeState.PULLBACK
                }
            }
            IntakeState.PULLBACK -> {
                runIntake(-1 * C.pullbackSpeed)
                if (!limitSwitch.get()) {
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
            IntakeState.OUTTAKING -> runIntake(C.outtakeSpeed)
            else -> stop()
        }
    }

}