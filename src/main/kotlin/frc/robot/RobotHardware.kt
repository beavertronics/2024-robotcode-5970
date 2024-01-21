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
    class Drive : SubsystemBase() {
        private val        left = CANSparkMax(21,MotorType.kBrushless)
        private val  leftFollow = CANSparkMax(22,MotorType.kBrushless)
        private val       right = CANSparkMax(23,MotorType.kBrushless)
        private val rightFollow = CANSparkMax(24,MotorType.kBrushless)

        //Run a piece of code for each drive motor controller.
        fun allMotors(code: CANSparkMax.() -> Unit) {
            for (motor in listOf(left, right)) {
                motor.apply(code)
            }
        }

        val drive = DifferentialDrive(left, right)
        val imu =  AHRS()

        init {

            leftFollow.follow(left)
            rightFollow.follow(right)

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
    val drive = Drive()
}