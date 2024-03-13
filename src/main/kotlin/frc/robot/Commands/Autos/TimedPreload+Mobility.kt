package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.*
import frc.robot.Constants

/**
 * Runs shootNoteOpenLoop (Ignore the jank)
 */
class `TimedPreload+Mobility`(
    private val backupVoltage : Double = -4.0,
    private val backupTime : Double = 0.4,
    private val spinupSpeed : Double = 1.0,
    private val spinupTime : Double = 1.0,
    private val secondBackupVoltage : Double = -5.0,
    private val secondBackupTime : Double = 1.0,

) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            DrivetrainControl.runDrivetrain(backupVoltage,backupTime),
            ShootNoteOpenLoop(spinupSpeed, spinupTime),
            DrivetrainControl.runDrivetrain(secondBackupVoltage,secondBackupTime),
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}