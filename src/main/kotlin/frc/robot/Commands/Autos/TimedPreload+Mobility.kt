package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.*

/**
 * Runs shootNoteOpenLoop (Ignore the jank)
 */
class `TimedPreload+Mobility`(
    private val backupVoltage : Double = -4.0,
    private val backupTime : Double = 0.3,
    private val spinupSpeed : Double = 1.0,
    private val spinupTime : Double = 1.0,
    private val secondBackupVoltage : Double = -5.0,
    private val secondBackupTime : Double = 1.0,
    private val waitTime : Double = 1.0


) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
                BasicControl.Wait(0.5),
                IntakeControl.Outtake(0.3,0.3),
            DrivetrainControl.runDrivetrain(backupVoltage,backupTime),
            ShootNoteOpenLoop(spinupSpeed, spinupTime),
                BasicControl.Wait(waitTime),
            DrivetrainControl.runDrivetrain(secondBackupVoltage,secondBackupTime),
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}