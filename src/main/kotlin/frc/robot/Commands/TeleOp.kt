package frc.robot.commands

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.Sugar.within
import frc.robot.Constants

import frc.robot.subsystems.Intake
import kotlin.math.*

import frc.robot.subsystems.Drivetrain


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {



    override fun initialize() {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.rawCurvatureDrive(OI.throttle, OI.turn)
        when {
            OI.intake -> Intake.runIntakeRaw(Constants.IntakeConstants.voltage)
            OI.outtake -> Intake.runIntakeRaw(-Constants.IntakeConstants.voltage)
            else -> Intake.stop()
        }
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
        public val intake get() = operatorController.getRawButton(1) //Todo: Get right button
        public val outtake get() = operatorController.getRawButton(2) //Todo: Get right button
        //TODO: Bring back this code- quickturns!
        //val quickTurnRight    get() = driverController.rightTriggerAxis
        //val quickTurnLeft     get() = driverController.leftTriggerAxis

        //TODO: Increased speed trigger for zipping across the field?
    }
}