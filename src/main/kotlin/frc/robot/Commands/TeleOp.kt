package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.Joystick

import frc.engine.utils.Sugar.within
import kotlin.math.*

import frc.robot.RobotHardware


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {


    object OperatorInterface {
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
    
        val turn: Double get() = driverController.leftX.processInput(squared = true)
        val throttle: Double get() = driverController.leftY.processInput(squared = true)
        val quickTurnRight get() = driverController.rightTriggerAxis
        val quickTurnLeft get() = driverController.leftTriggerAxis
    }

    val bot = RobotHardware.Drive

    override fun initialize() {
        addRequirements(RobotHardware.Drive)
    }

    override fun execute() {
        
    }


}