package frc.robot.Commands.Autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Commands.Basic.DrivetrainControl.followPathCommand
import frc.robot.Commands.Basic.ShootNote
import frc.robot.Constants
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class `Preload+Mobility`  : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            ShootNote(Constants.ShooterConstants.SpeakerSpeed),
            followPathCommand("BottomSpeakerToMobility")
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }
}