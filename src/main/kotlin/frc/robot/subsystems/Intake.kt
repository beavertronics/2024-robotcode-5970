package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.DigitalInput
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    val limitSwitch = DigitalInput(C.limitSwitchChannel)

    private val topMotor = TalonSRX(C.TopMotorID)
    private val bottomMotor = TalonSRX(C.BottomMotorID)

    init {
        initMotorControllers(C.CurrentLimit, topMotor, bottomMotor)
        /*TopMotor.setSmartCurrentLimit(C.CurrentLimit)
        TopMotor.restoreFactoryDefaults()
        bottomMotor.configContinuousCurrentLimit(C.CurrentLimit)
        bottomMotor.configFactoryDefault()*/
        bottomMotor.inverted = true
        topMotor.inverted = true

        bottomMotor.follow(topMotor)


    }

    /** Runs the intake motor at the given percentage
     * @param speed The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntake(speed:Double) {
        //bottomMotor.set(ControlMode.PercentOutput, speed)
        topMotor.set(ControlMode.PercentOutput, speed)
    }
    /** Runs the intake motor at 0%, stopping it */
    fun stop() {
        //bottomMotor.set(ControlMode.PercentOutput, 0.0)
        topMotor.set(ControlMode.PercentOutput, 0.0)
    }



    fun doIntake() : Command = 
        this.run {runIntake(C.pickupSpeed)}
        //.until{!limitSwitch.get()}
        //.andThen(this.run{runIntake(C.pickupSpeed)})
        //.onlyWhile{limitSwitch.get()}

    fun doFeed() : Command = this.run{runIntake(C.feedingSpeed)}

    fun doEject()  : Command = this.run { runIntake(-C.reverseSpeed) }

    fun idle() : Command = this.run { stop() }.withName("Idle")

    init {
        defaultCommand = idle()
    }
}