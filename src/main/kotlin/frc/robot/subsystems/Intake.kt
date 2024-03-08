package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.engine.utils.initMotorControllers
import frc.robot.Constants.IntakeConstants.unfeedTime
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {

    val limitSwitch = DigitalInput(C.limitSwitchChannel)

    private val topMotor = TalonSRX(C.TopMotorID)
    private val bottomMotor = TalonSRX(C.BottomMotorID)

    private val unFeedTimer = Timer()

    init {
        // Reset motor controllers & set current limits
        initMotorControllers(C.CurrentLimit, topMotor, bottomMotor)

        // Sets the bottom motor to follow the top (as they should never be running seperatly)
        bottomMotor.follow(topMotor)

        // Invert the top & bottom controlers
        bottomMotor.inverted = true
        topMotor.inverted = true

    }

    /** Runs the intake motor at the given percentage
     * @param speed The voltage to run the motor at. Positive is intake, Negative is outake
     */
    fun runIntake(speed:Double) {
        topMotor.set(ControlMode.PercentOutput, speed)
    }
    /** Runs the intake motor at 0%, stopping it */
    fun stop() {
        topMotor.set(ControlMode.PercentOutput, 0.0)
    }

}