package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.*
import frc.robot.Constants

/**
 * Runs shootNoteOpenLoop (Ignore the jank)
 */
class `TimedPreload+Mobility` : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            ShootNoteOpenLoop(1.0, 1.0),
            OHGODTHEYGAVEUS2MINUTESTOTESTATCOMP_auto()
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}