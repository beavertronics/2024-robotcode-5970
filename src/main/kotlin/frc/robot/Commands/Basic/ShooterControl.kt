package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.engine.utils.RPM
import frc.robot.subsystems.Shooter

object ShooterControl {
    /** Uses FF and PID to get the shoot up to the desired RPM, and ends when it is at speed
     * @param speed The desired speed of the shooter*/
    class SpinupShooter(
        private val speed: RPM
    ) : Command() {
        override fun initialize() =Shooter.setSpeed(speed)
        override fun execute() = Shooter.runClosedLoop()
        override fun isFinished(): Boolean { return Shooter.isAtSpeed}

    }
    /** Continues running shooter at whatever speed it was set to last
     * (Does not end unless interrupted) */
    class RunShooter() : Command() {
        override fun execute() = Shooter.runClosedLoop()
    }

    /**
     * Run the shooter at a certain percent until a certain amount of time has elapsed
     * @property percent The percent to run the shooter at
     * @property time The time to run the shooter for
     */
    class OpenLoopSpinup(
        private val percent: Double,
        private val time   : Double
    ) : Command() {
        private val timer = Timer()
        override fun initialize() = timer.restart()
        override fun execute() =    Shooter.runOpenLoop(percent)
        override fun isFinished(): Boolean { return timer.hasElapsed(time)}

    }
    /** Continues running shooter at percent
     * (Does not end unless interrupted) */
    class RunOpenloop(val percent : Double) : Command() {
        override fun execute() = Shooter.runOpenLoop(percent)
    }
    class StopShooter() : Command() {
        override fun initialize() = Shooter.stop()
        override fun isFinished(): Boolean {return true}
    }

}
