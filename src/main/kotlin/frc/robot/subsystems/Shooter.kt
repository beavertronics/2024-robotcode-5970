package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.RPM

object Shooter : SubsystemBase() {
    /**
     * Set the speed of the flywheels using closed loop control
     * @param speed Speed of the motor in RPM
     */
    fun setSpeed(speed : Double){

    }


    fun setSpeed(speed : RPM) { setSpeed((speed.rotationsPerMinute()))}
}