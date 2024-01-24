package frc.robot

import edu.wpi.first.wpilibj2.command.SubsystemBase

import edu.wpi.first.wpilibj.drive.DifferentialDrive

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel.MotorType

import com.kauailabs.navx.frc.AHRS


/* Object that represents the robot's hardware, organized into subsystems.

Code for *controlling* this hardware should be located elsewhere (ie in RobotController).
This should just include the low-level components, ie motor controllers.
Talk to Will Kam about why organizing the code this way will be beneficial.



*/

object RobotHardware {
    object Drive : SubsystemBase() {
        private val       leftMain = CANSparkMax(21,MotorType.kBrushless)
        private val  leftSecondary = CANSparkMax(22,MotorType.kBrushless)
        private val      rightMain = CANSparkMax(23,MotorType.kBrushless)
        private val rightSecondary = CANSparkMax(24,MotorType.kBrushless)

        //Run a piece of code for each drive motor controller.
        fun allMotors(code: CANSparkMax.() -> Unit) {
            for (motor in listOf(leftMain, rightMain)) {
                motor.apply(code)
            }
        }

        val drive = DifferentialDrive(leftMain, rightMain)
        val imu =  AHRS()

        init {

            leftSecondary.follow(leftMain)
            rightSecondary.follow(rightMain)

            allMotors {
                restoreFactoryDefaults()
                //TODO: initialize spark maxes
            }
            /* TODO: set motor inversion correctly
            leftMain.inverted = false
            leftSecondary.inverted = false
            rightMain.inverted = true
            rightSecondary.inverted = true
            */
        }
    }
    /*TODO: Intake Hardware
    class Intake : SubsystemBase() {
        t o d o
    }
    */
}