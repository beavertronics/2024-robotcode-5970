package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.*
import frc.robot.Constants
/*
    Working settings:
    private val backupVoltage : Double = -4.0,
    private val backupTime : Double = 0.2,
    private val spinupSpeed : Double = 1.0,
    private val spinupTime : Double = 1.0,
    private val preIntakeDriveForwardVoltage : Double = 2.0,
    private val preIntakeDriveForwardTime : Double = 0.1,
    private val intakeSpeed : Double = 0.65,
    private val intakeTime : Double = 1.3,
    private val intakeDrivetrainSpeed : Double = -3.0,
    private val driveForwardVoltage : Double = 4.0,
    private val driveForwardTime : Double = 0.5,
    private val waitTime: Double = 1.0,
 */
/**
 * Runs shootNoteOpenLoop (Ignore the jank)
 */
class TimedTwoNoteAuto(
    private val backupVoltage : Double = -4.0,
    private val backupTime : Double = 0.2,
    private val spinupSpeed : Double = 1.0,
    private val spinupTime : Double = 1.0,
    private val preIntakeDriveForwardVoltage : Double = 2.0,
    private val preIntakeDriveForwardTime : Double = 0.1,
    private val intakeSpeed : Double = 0.65,
    private val intakeTime : Double = 1.3,
    private val intakeDrivetrainSpeed : Double = -3.0,
    private val driveForwardVoltage : Double = 4.0,
    private val driveForwardTime : Double = 0.5,
    private val waitTime: Double = 1.0,



    ) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
                DrivetrainControl.runDrivetrain(backupVoltage,backupTime),
                ShootNoteOpenLoop(spinupSpeed, spinupTime),
                BasicControl.Wait(waitTime),
                DrivetrainControl.runDrivetrain(preIntakeDriveForwardVoltage,preIntakeDriveForwardTime),
                BasicControl.Wait(waitTime),
                IntakeNoteNoOutake(intakeSpeed,intakeDrivetrainSpeed,intakeTime),
                IntakeControl.TimedPickup(intakeSpeed,waitTime),
                IntakeControl.Outtake(time=0.2),
                DrivetrainControl.runDrivetrain(driveForwardVoltage,driveForwardTime),
                ShootNoteOpenLoop(spinupSpeed, spinupTime),
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}