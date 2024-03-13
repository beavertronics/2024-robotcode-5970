package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

object BasicControl {
    class Wait(private val time : Double) : Command() {
        val timer = Timer()
        override fun initialize() = timer.restart()
        override fun isFinished(): Boolean {return timer.hasElapsed(time) }

    }
    class RobotFullStop : Command() {
        override fun initialize() {
            Shooter.stop()
            Intake.stop()
            Drivetrain.stop()
        }

        override fun isFinished(): Boolean { return true}
    }
}