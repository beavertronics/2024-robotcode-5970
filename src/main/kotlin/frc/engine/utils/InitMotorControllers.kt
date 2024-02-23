package frc.engine.utils

import com.revrobotics.CANSparkMax

fun initMotorControllers(currentLimit : Int, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
    }
}