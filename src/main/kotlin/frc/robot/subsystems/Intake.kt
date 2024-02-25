package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    val limitSwitch = DigitalInput(C.limitSwitchChannel)

    private val TopMotor = TalonSRX(C.TopMotorID)
    private val bottomMotor = TalonSRX(C.BottomMotorID)

    init {
        initMotorControllers(C.CurrentLimit, TopMotor, bottomMotor)
        /*TopMotor.setSmartCurrentLimit(C.CurrentLimit)
        TopMotor.restoreFactoryDefaults()
        bottomMotor.configContinuousCurrentLimit(C.CurrentLimit)
        bottomMotor.configFactoryDefault()*/
        bottomMotor.inverted = true
        TopMotor.inverted = false

        bottomMotor.follow(TopMotor)


    }

    /** Runs the intake motor at the give voltage
     * @param voltage The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntakeRaw(voltage:Double) {
        //bottomMotor.set(ControlMode.PercentOutput, voltage/12)
        TopMotor.set(ControlMode.PercentOutput,voltage/12)
    }
    /** Runs the intake motor at 0V, stopping it
     */
    fun stop() {
        //bottomMotor.set(ControlMode.PercentOutput, 0.0)
        TopMotor.set(ControlMode.PercentOutput, 0.0)
    }
    fun runIntake(speed:Double) {
        //bottomMotor.set(ControlMode.PercentOutput, speed)
        TopMotor.set(ControlMode.PercentOutput, speed)
    }


    fun doIntake() : Command = 
        this.run {runIntake(C.pickupSpeed)}
        .onlyWhile{!limitSwitch.get()}
        .andThen(this.run{runIntake(C.pickupSpeed)})
        .onlyWhile{limitSwitch.get()}

    fun doFeed() : Command = this.run{runIntake(C.feedingSpeed)}

    fun doEject()  : Command = this.run { runIntake(-C.reverseSpeed) }

    fun idle() : Command = this.run { stop() }.withName("Idle")

    init {
        defaultCommand = idle()
    }
}