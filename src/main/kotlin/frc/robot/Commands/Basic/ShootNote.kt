package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.engine.utils.RPM
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class ShootNote(
    private val speed: RPM
) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            Shooter.spinup(speed),
            Intake.feed()
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

}