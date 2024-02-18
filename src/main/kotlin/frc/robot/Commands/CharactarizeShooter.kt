package frc.robot.Commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.Shooter

var routine = SysIdRoutine(
    SysIdRoutine.Config(),
    SysIdRoutine.Mechanism(
        Shooter.rawDrive,
        Shooter.logger,
        Shooter
    )
)
fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command? {
    return routine.quasistatic(direction)
}

fun sysIdDynamic(direction: SysIdRoutine.Direction): Command? {
    return routine.dynamic(direction)
}

class ID_THAT_SYS : Command() {
    private lateinit var autoCommandGroup: Command

    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            sysIdDynamic(SysIdRoutine.Direction.kForward),
            sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

}