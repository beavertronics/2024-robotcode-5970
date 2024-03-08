package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.RPM
import frc.robot.Constants
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

object IntakeControl {
    /** Runs the intake at speed for time
     * @property speed The speed to run the intake at
     * @property time The time to run the intake for */
    class Feed(
        private val speed: Double = Constants.IntakeConstants.feedingSpeed,
        private val time: Double = Constants.IntakeConstants.feedingTime
    ) : Command() {
        private val timer = Timer()
        override fun initialize() { timer.restart() }
        override fun execute() = Intake.runIntake(speed)
        override fun isFinished(): Boolean { return timer.hasElapsed(time)}

    }
    class Pickup (
        private val speed: Double = Constants.IntakeConstants.pickupSpeed
    ) : Command() {
        override fun execute() = Intake.runIntake(speed)
    }
}
