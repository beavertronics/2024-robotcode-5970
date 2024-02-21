package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.Constants
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    public val limitSwitch = DigitalInput(C.limitSwitchChannel)

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

    fun doIntake() : Command = this.pickup().andThen(this.pullBack().withInterruptBehavior(kCancelIncoming))

    fun pickup()   : Command = this.run({ runIntake(C.pickupSpeed)  }).onlyWhile({ !limitSwitch.get() }).withName("Pickup")
    fun pullBack() : Command = this.run({ runIntake(C.pullbackSpeed) }).onlyWhile({ limitSwitch.get() }).withName("Pull Back")

    fun feed() : Command = this.run({ runIntake(C.feedingSpeed) }).withName("Feed to Shooter")

    init {
        defaultCommand = this.run({ stop() }).withName("Idle")
    }
}