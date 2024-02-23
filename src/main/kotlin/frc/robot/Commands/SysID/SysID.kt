package frc.robot.Commands.SysID

import edu.wpi.first.units.Distance
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.MutableMeasure.mutable
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.subsystems.Drivetrain
//import frc.robot.subsystems.Drivetrain.logger
import frc.robot.subsystems.Drivetrain.rawDrive


/*var routine = SysIdRoutine(
    SysIdRoutine.Config(),
    Mechanism(
        rawDrive,
        logger,
        Drivetrain
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

}*/