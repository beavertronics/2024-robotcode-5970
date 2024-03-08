package frc.robot.Commands.Basic

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
}
