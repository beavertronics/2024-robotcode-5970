package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.RPM
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

object IntakeControl {
    /** Runs the intake at speed for time
     * @property speed The speed to run the intake at
     * @property time The time to run the intake for */
    class Feed(
        private val speed: Double = Intake.FEEDING_SPEED,
        private val time: Double = Intake.FEEDING_SPEED
    ) : Command() {
        private val timer = Timer()
        override fun initialize() = timer.restart()
        override fun execute() = Intake.runIntake(speed)
        override fun end(interrupted: Boolean) = Intake.stop()
        override fun isFinished(): Boolean { return timer.hasElapsed(time)}

    }
    class Pickup (
        private val speed: Double = Intake.PICKUP_SPEED
    ) : Command() {
        override fun execute() = Intake.runIntake(speed)
        override fun end(interrupted: Boolean) = Intake.stop()
    }
    class TimedPickup (
        private val speed: Double = Intake.PICKUP_SPEED,
        private val time: Double = 1.0
    ) : Command() {
        private val timer = Timer()
        override fun initialize() = timer.restart()
        override fun execute() = Intake.runIntake(speed/4,speed)
        override fun end(interrupted: Boolean) = Intake.stop()
        override fun isFinished(): Boolean { return timer.hasElapsed(time)}


    }
    class Outtake (
            private val speed: Double = 0.3,
            private val time: Double = 1.0
    ) : Command() {
        private val timer = Timer()
        override fun initialize() = timer.restart()
        override fun execute() = Intake.runIntake(-speed)
        override fun end(interrupted: Boolean) = Intake.stop()
        override fun isFinished(): Boolean { return timer.hasElapsed(time)}

    }
}
