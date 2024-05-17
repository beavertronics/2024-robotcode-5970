package frc.robot

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import frc.engine.utils.initMotorControllers
import frc.robot.Safety.DriveConstants
import frc.robot.Safety.IntakeConstants
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Drivetrain.drive
import frc.robot.subsystems.Intake

object Hardware {
    //Motor Controller CAN IDs!
    private val       leftMain = CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless)
    private val  leftSecondary = CANSparkMax(22,  CANSparkLowLevel.MotorType.kBrushless)
    private val      rightMain = CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSecondary = CANSparkMax(24,  CANSparkLowLevel.MotorType.kBrushless)
    private val   leftFlywheel = CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless)
    private val  rightFlywheel = CANSparkMax(26, CANSparkLowLevel.MotorType.kBrushless)
    private val    bottomMotor = TalonSRX(27)
    private val       topMotor = TalonSRX(28) //CAN IDs

    val limitSwitch = DigitalInput(1)

    init {
        // Reset motor controllers & set current limits in AMPS
        initMotorControllers(20, topMotor, bottomMotor) //Shooter
        initMotorControllers(20, leftFlywheel, rightFlywheel) //Intake
        initMotorControllers(30, leftMain, leftSecondary, rightMain, rightSecondary) //Drivetrain

        // Invert the top & bottom motors (shooter)
        bottomMotor.inverted = true
        topMotor.inverted = true

        // Invert the left motors (drivetrain
        leftMain.inverted = true
        leftSecondary.inverted = true

    }
}