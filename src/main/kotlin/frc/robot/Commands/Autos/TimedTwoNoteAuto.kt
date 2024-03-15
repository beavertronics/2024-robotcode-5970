package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.*
import frc.robot.Constants

/**
 * Runs shootNoteOpenLoop (Ignore the jank)
 */
class TimedTwoNoteAuto(
    private val backupVoltage : Double = -4.0,
    private val backupTime : Double = 0.3,
    private val spinupSpeed : Double = 1.0,
    private val spinupTime : Double = 1.0,
    private val intakeSpeed : Double = 0.65,
    private val intakeTime : Double = 0.8,
    private val intakeDrivetrainSpeed : Double = -3.0,
    private val driveForwardVoltage : Double = 4.0,
    private val driveForwardTime : Double = 0.4,
    private val waitTime: Double = 1.0,



    ) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
                DrivetrainControl.runDrivetrain(backupVoltage,backupTime),
                ShootNoteOpenLoop(spinupSpeed, spinupTime),
                BasicControl.Wait(waitTime),
                IntakeNote(intakeSpeed,intakeDrivetrainSpeed,intakeTime),
                DrivetrainControl.runDrivetrain(driveForwardVoltage,driveForwardTime),
                ShootNoteOpenLoop(spinupSpeed, spinupTime),
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}