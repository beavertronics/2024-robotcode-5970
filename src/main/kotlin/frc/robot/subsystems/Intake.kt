package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.engine.utils.initMotorControllers
import frc.robot.Safety.IntakeConstants as C

object Intake : SubsystemBase() {

    val limitSwitch = DigitalInput(C.limitSwitchChannel)


    private val unFeedTimer = Timer()
    private val    topMotor = TalonSRX(28) //CAN IDs
    private val bottomMotor = TalonSRX(27)

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(C.CURRENT_LIMIT, topMotor, bottomMotor)

        // Sets the bottom motor to follow the top (as they should never be running seperatly)
        //bottomMotor.follow(topMotor)

        // Invert the top & bottom controlers
        bottomMotor.inverted = true
        topMotor.inverted = true

    }

    /** Runs the intake motor at the given percentage
     * @param speed The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntake(speed:Double) {
        topMotor.set(ControlMode.PercentOutput, speed)
        bottomMotor.set(ControlMode.PercentOutput, speed)
    }
    /** Runs the intake motor at the given percentage
     * @param speed The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntake(topSpeed:Double, bottomSpeed: Double) {
        topMotor.set(ControlMode.PercentOutput, topSpeed)
        bottomMotor.set(ControlMode.PercentOutput, bottomSpeed)
    }
    /** Runs the intake motor at 0%, stopping it */
    fun stop() {
        topMotor.set(ControlMode.PercentOutput, 0.0)
        bottomMotor.set(ControlMode.PercentOutput, 0.0)

    }
    const val PICKUP_SPEED = 0.7
    const val PUSHFORWOARD_SPEED = 0.3
    const val REVERSE_SPEED = 0.5
    const val FEEDING_SPEED = 0.7

    /* Old intake time constants from auto-intake system
    const val feedingTime = 1.0 //TODO set feedingTime. In seconds
    const val unfeedTime = 0.5 //TODO set feedingTime. In seconds

    val limitSwitch = DigitalInput(1) //LIMIT SWITCH CHANNEL ON ROBORIO DIO

    private val unFeedTimer = Timer()
    */
}