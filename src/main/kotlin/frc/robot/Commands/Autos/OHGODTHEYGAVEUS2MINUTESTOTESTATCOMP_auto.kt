package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

class OHGODTHEYGAVEUS2MINUTESTOTESTATCOMP_auto (
        private val time: Double = 2.0,
        private val voltage: Double = -4.0
): Command() {
    val timer = Timer()
    override fun initialize() = timer.restart()
    override fun execute() = Drivetrain.voltageDrive(voltage,voltage)

    override fun isFinished(): Boolean { return timer.hasElapsed(time) }

    override fun end(interrupted: Boolean) = Drivetrain.stop()
}