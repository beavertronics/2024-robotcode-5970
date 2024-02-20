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
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    private val limitSwitch = DigitalInput(C.limitSwitchChannel)

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
    private val feedingTimer = Timer()
    var intakeSpeed = C.pickupSpeed
    var outtakeSpeed = C.reverseSpeed


    /** Run the intake at intakeSpeed until a limitswitch is pressed,
     * then runs intake at -pullbackSpeed until limitswitch is no longer pressed */
    fun doIntake() : Command = this.pickup().andThen(this.pullBack().andThen(loaded()))

    /** Runs the intake at intakeSpeed speed */
    fun pickup()   : Command =
            this.run { runIntake(intakeSpeed) }
                    .onlyWhile { !limitSwitch.get() }
                    .withName("PICKUP")
    /** Runs the intake at -pullbackSpeed speed */
    fun pullBack() : Command =
            this.run { runIntake(-C.pullbackSpeed) }
                    .onlyWhile { limitSwitch.get() }
                    .withName("PULL_BACK")
    /** Runs the intake at feedingSpeed speed for feedingTime seconds */
    fun feed()   : Command =
            this.run { runIntake(C.feedingSpeed) }
                    .beforeStarting( {feedingTimer.reset(); feedingTimer.start()} )
                    .onlyWhile { !feedingTimer.hasElapsed(C.feedingTime) }
                    .andThen(idle())
                    .withName("FEEDING")
                    .withInterruptBehavior(kCancelIncoming)
    /** Runs the intake at outtakeSpeed speed */
    fun outtake()   : Command =
            this.run { runIntake(-outtakeSpeed) }
                    .beforeStarting( {feedingTimer.reset(); feedingTimer.start()} )
                    .withName("OUTTAKE")
    /** Stops the intake motors */
    fun idle() : Command =
            this.run { stop() }.withName("IDLE")
    /** Stops the intake motors, stops intake command in Teleop */
    fun loaded() : Command =
            this.run { stop() }.withName("LOADED")



    init {
        defaultCommand = idle()
    }
}