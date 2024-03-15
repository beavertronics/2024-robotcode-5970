package frc.robot.Commands.Basic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.engine.utils.RPM
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class IntakeNote(
    private val intakeSpeed: Double,
        private val drivetrainSpeed: Double,
        private val time: Double
) : Command() {
    private lateinit var autoCommandGroup : SequentialCommandGroup
    private val timer = Timer()
    override fun initialize() {
        timer.restart()
        autoCommandGroup = SequentialCommandGroup (
                ParallelDeadlineGroup (
                        BasicControl.Wait(time),
                        IntakeControl.TimedPickup(intakeSpeed,time),
                        DrivetrainControl.runDrivetrain(drivetrainSpeed,time)
                ),
                IntakeControl.Outtake(0.3,0.2)

        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished || timer.hasElapsed(time)
    }

}
