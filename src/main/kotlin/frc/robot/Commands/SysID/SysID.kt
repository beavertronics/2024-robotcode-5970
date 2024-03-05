package frc.robot.Commands.SysID

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter

val shooterSys = SysIdRoutine(
    SysIdRoutine.Config(),
    SysIdRoutine.Mechanism(
        Shooter.rawDrive,
        Shooter.logger,
        Shooter
    )
)

val drivetrainSys = SysIdRoutine(
    SysIdRoutine.Config(),
    SysIdRoutine.Mechanism(
        Drivetrain.voltageDrive,
        Drivetrain.logger,
        Drivetrain
    )
)

class SysID(private val routine: SysIdRoutine) : Command() {
    private lateinit var autoCommandGroup: Command

    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            routine.dynamic(SysIdRoutine.Direction.kReverse)
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

}