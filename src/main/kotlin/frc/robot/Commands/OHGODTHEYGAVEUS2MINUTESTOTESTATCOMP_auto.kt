package frc.robot.Commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

class OHGODTHEYGAVEUS2MINUTESTOTESTATCOMP_auto : Command() {
    val timer = Timer()
    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        Drivetrain.rawDrive(5.0,5.0)
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(3.0)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}