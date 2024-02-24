package frc.engine.utils

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax

fun initMotorControllers(currentLimit : Int, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
    }
}
fun initMotorControllers(currentLimit : Int, vararg  motors : TalonSRX){
    motors.forEach {
        it.configFactoryDefault()
        it.configContinuousCurrentLimit(currentLimit)
    }
}