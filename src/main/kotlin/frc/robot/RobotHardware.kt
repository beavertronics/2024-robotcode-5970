package frc.robot

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.controller.PIDController

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel.MotorType

import com.kauailabs.navx.frc.AHRS as NAVX

import frc.robot.Constants.DriveConstants as D

/* Object that represents the robot's hardware, organized into subsystems.

Code for *controlling* this hardware should be located elsewhere (ie in RobotController).
This should just include the low-level components, ie motor controllers.
Talk to Will Kam about why organizing the code this way will be beneficial.



*/

object RobotHardware {

    object Drive : SubsystemBase() {
        private val       leftMain = CANSparkMax(D.MotorRMainID,MotorType.kBrushless)
        private val  leftSecondary = CANSparkMax(D.MotorRSubID,MotorType.kBrushless)
        private val      rightMain = CANSparkMax(D.MotorRMainID,MotorType.kBrushless)
        private val rightSecondary = CANSparkMax(D.MotorRSubID,MotorType.kBrushless)

        val drive = DifferentialDrive(leftMain, rightMain)
        val imu =  NAVX()

        fun allMotors(code: CANSparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
            for (motor in listOf(leftMain, rightMain)) {
                motor.apply(code)
            }
        }

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