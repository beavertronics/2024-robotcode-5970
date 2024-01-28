package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.IntakeConstants as C

object Intake : SubsystemBase() {
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

}