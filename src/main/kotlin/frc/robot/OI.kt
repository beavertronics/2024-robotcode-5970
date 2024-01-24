package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.Sugar.within
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

object OI : SubsystemBase() {
    private val driverController    = XboxController(0)
    private val operatorController  = Joystick(1)

    private fun Double.processInput(deadzone : Double = 0.1, squared : Boolean = false, cubed : Boolean = false, readjust:Boolean = true) : Double{
        var processed = this
        if(readjust) processed = ((this.absoluteValue-deadzone)/(1-deadzone))*this.sign
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