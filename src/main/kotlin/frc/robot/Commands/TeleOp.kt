package frc.robot.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.Sugar.within
import frc.robot.Constants

import kotlin.math.*

import frc.robot.subsystems.Drivetrain
import frc.robot.RobotController


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {
    private val       motor = VictorSPX(27)
    override fun initialize() {
        SmartDashboard.putNumber("MaxChildSpeed", 0.35)
        SmartDashboard.putNumber("MaxChildTurningSpeed", 0.4)
        addRequirements(Drivetrain)
    }

    override fun execute() {
        motor.set(ControlMode.PercentOutput, 0.3)


    }

}