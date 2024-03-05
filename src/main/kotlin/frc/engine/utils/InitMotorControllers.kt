package frc.engine.utils

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkMax

/**
 * Restores the factory defaults for the given spark motor controllers, and sets the current limits
 * @param currentLimit Current limit to set the motors to
 */
fun initMotorControllers(currentLimit : Int, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
    }
}
/**
 * Restores the factory defaults for the given spark motor controllers,
 * and sets the current limits & idle mode
 * @param currentLimit Current limit to set the motors to
 * @param idle The idle mode to set the controller to
 */
fun initMotorControllers(currentLimit : Int, idle: CANSparkBase.IdleMode, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
        it.idleMode = idle
    }
}
/**
 * Restores the factory defaults for the given TalonSRX controllers, and sets the current limits
 * @param currentLimit Current limit to set the motors to
 */
fun initMotorControllers(currentLimit : Int, vararg  motors : TalonSRX){
    motors.forEach {
        it.configFactoryDefault()
        it.configContinuousCurrentLimit(currentLimit)
    }
}
/**
 * Restores the factory defaults for the given TalonSRX controllers, and sets the current limits
 * @param currentLimit Current limit to set the motors to
 * @param idle The idle mode to set the controller to
 */
fun initMotorControllers(currentLimit : Int, idle: NeutralMode, vararg  motors : TalonSRX){
    motors.forEach {
        it.configFactoryDefault()
        it.configContinuousCurrentLimit(currentLimit)
        it.setNeutralMode(idle)
    }
}
