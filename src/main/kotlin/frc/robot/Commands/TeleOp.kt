package frc.robot.commands

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.Sugar.within

import kotlin.math.*

import frc.robot.subsystems.Drivetrain


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    enum class DriveMode {
        DEFAULT,CHILD
    }
    private var driveMode = DriveMode.DEFAULT
    override fun initialize() {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        when (driveMode) {
            DriveMode.DEFAULT -> Drivetrain.percentCurvatureDrive(OI.throttle*0.5, OI.turn*0.5)
            
            DriveMode.CHILD -> {
                if (OI.rTrigger) {
                    // Safe for kid to drive. Uses flight joysticks
                    Drivetrain.percentDrive(OI.leftJoystick*0.25, OI.rightJoystick*0.25)
                } else {
                    // Not safe. Drives with xbox controller
                    Drivetrain.percentCurvatureDrive(OI.throttle*0.5, OI.turn*0.5)
                }
            }
        }

    }

    object OI {
        private val driverController    = XboxController(0)
        private val lOpControl  = Joystick(1)
        private val rOpControl  = Joystick(2)


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
        public val leftJoystick get() = lOpControl.getRawAxis(2)
        public val rightJoystick get() = rOpControl.getRawAxis(2)

        //TODO: Bring back this code- quickturns!
        val rTrigger    get() = driverController.rightTriggerAxis.abs_GreaterThan(0.1)
        //val quickTurnLeft     get() = driverController.leftTriggerAxis

        //TODO: Increased speed trigger for zipping across the field?
    }
}0