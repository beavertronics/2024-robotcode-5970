package frc.robot.commands

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.Sugar.within


import frc.robot.Constants.TeleopConstants
import kotlin.math.*

import frc.robot.subsytems.Drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive

//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    override fun initialize() {
        addRequirements(Drivetrain /* TODO: Require shooter/intake/climb subsystems! */)  
    }

    override fun execute() {
        val speeds = DifferentialDrive.curvatureDriveIK(OI.throttle, OI.turn, true)
        Drivetrain.rawDrive(speeds.left * TeleopConstants.MaxVoltage, speeds.right * TeleopConstants.MaxVoltage)
        //TODO: Tune drive!

        //TODO: Control Subsystems!
    }

    object OI {
        private val driverController    = XboxController(0)
        private val operatorController  = Joystick(1)

        private fun Double.processInput(deadzone : Double = 0.1, squared : Boolean = false, cubed : Boolean = false, readjust : Boolean = true) : Double{
            var processed = this
            if(readjust) processed = ((this.absoluteValue - deadzone)/(1 - deadzone))*this.sign
            return when {
                this.within(deadzone) ->    0.0
                squared ->                  processed.pow(2) * this.sign
                cubed ->                    processed.pow(3)
                else ->                     processed
            }
        }
        private fun Double.abs_GreaterThan(target: Double): Boolean{
            return this.absoluteValue > target
        }

        public val turn get() = driverController.leftX.processInput(squared = true)
        public val throttle get() = driverController.leftY.processInput(squared = true)

        //TODO: Bring back this code- quickturns!
        //val quickTurnRight    get() = driverController.rightTriggerAxis
        //val quickTurnLeft     get() = driverController.leftTriggerAxis

        //TODO: Increased speed trigger for zipping across the field?
    }
}