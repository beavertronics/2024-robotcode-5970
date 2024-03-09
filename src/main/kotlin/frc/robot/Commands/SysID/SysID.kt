package frc.robot.Commands.SysID

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter

val shooterSys = SysIdRoutine(
    SysIdRoutine.Config(
    ),
    SysIdRoutine.Mechanism(
        Shooter.rawDrive,
        Shooter.logger,
        Shooter
    )
)
fun test(){
    PWMSparkMax(1)
    Encoder(0,1,true).rate
}
val drivetrainSys = SysIdRoutine(
    SysIdRoutine.Config(),
    SysIdRoutine.Mechanism(
        Drivetrain.voltageDrive,
        Drivetrain.logger,
        Drivetrain
    )
)
class PauseShooter(
        val time: Double = 1.0
) : Command(){
    val timer = Timer()
    override fun initialize() {
        timer.restart()
        Shooter.stop()
        Shooter.breakMotors()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(1.0)
    }
    override fun end(interrupted: Boolean) {
        Shooter.unBreakMotors()
    }
}

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
class SysIDShooter(private val routine: SysIdRoutine = shooterSys) : Command() {
    private lateinit var autoCommandGroup: Command

    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
                routine.quasistatic(SysIdRoutine.Direction.kForward),
                PauseShooter(),
                routine.quasistatic(SysIdRoutine.Direction.kReverse),
                PauseShooter(),
                routine.dynamic(SysIdRoutine.Direction.kForward),
                PauseShooter(),
                routine.dynamic(SysIdRoutine.Direction.kReverse)
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

}