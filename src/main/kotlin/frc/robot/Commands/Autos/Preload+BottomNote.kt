package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.DrivetrainControl.followPathCommand
import frc.robot.Commands.Basic.IntakeControl
import frc.robot.Commands.Basic.ShootNote
import frc.robot.Constants
import frc.robot.subsystems.Intake

class `Preload+BottomNote` : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {

        autoCommandGroup = SequentialCommandGroup (
            ShootNote(Constants.ShooterConstants.SpeakerSpeed),
            ParallelRaceGroup(
                followPathCommand("BottomSpeakerToBottomNote"),
                IntakeControl.Pickup()
            ),
            followPathCommand("BottomNoteToBottomSpeaker"),
            ShootNote(Constants.ShooterConstants.SpeakerSpeed)
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}