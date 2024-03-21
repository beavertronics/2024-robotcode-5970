package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants
import frc.robot.Constants.ClimbConstants.ClimbPos

import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.initMotorControllers

object Climber : SubsystemBase() {
    private val leftMotor  = CANSparkMax(ClimbConstants.LeftMotorID, MotorType.kBrushed) //TODO: Are we going to use NEOs, or 775s with encoders?
    private val rightMotor = CANSparkMax(ClimbConstants.RightMotorID, MotorType.kBrushed)

    init {
        // Reset motor controllers, set current limits, and set idle mode to brake
        initMotorControllers(ClimbConstants.CurrentLimit, CANSparkBase.IdleMode.kBrake, leftMotor, rightMotor)

        // Invert the right motor
        rightMotor.inverted = true
        leftMotor.inverted = false
    }

    /**
     * Runs the climber motors at left and right voltages, respectively
     * @param leftVolts The voltage to run the left motors at
     * @param rightVolts The voltage to run the right motors at
     */
    fun setVoltage(leftVolts: Double, rightVolts: Double) {

        leftMotor.setVoltage(leftVolts)
        rightMotor.setVoltage(rightVolts)
    }
    /**
     * Runs the climber motors at left and right voltages, respectively
     * @param voltage The voltage to run both the left & right motors at
     */
    fun setVoltage(voltage: Double) = setVoltage(voltage, voltage)
    fun stop() = setVoltage(0.0)

    /**
     * Runs the climber toward ClimbPos
     */
    fun climb(pos : ClimbPos) {
        when (pos) {
            ClimbPos.Retract -> setVoltage(-ClimbConstants.RetractVoltage)
            ClimbPos.Extend ->  setVoltage(ClimbConstants.ExtendVoltage)
            ClimbPos.Chill ->   setVoltage(0.0)
        }
        //TODO: does not make outo happy
    }
}