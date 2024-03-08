package frc.robot.Commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.engine.utils.RPM
import frc.engine.utils.Sugar.clamp
import frc.engine.utils.Sugar.within
import frc.robot.Constants

import frc.robot.Constants.TeleopConstants as C
import frc.robot.subsystems.*
import kotlin.math.*


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    override fun initialize() {
        addRequirements(Drivetrain,Intake,Shooter)
        
    }
    fun generatePath(dst: Pose2d): Trajectory {
        return Drivetrain.trajectoryMaker.builder()
            .start(Odometry.pose)
            .end(dst)
            .build()
    }
    /*
    Runs the code roughly every 0.02 seconds
    Used to control the robot
     */

    override fun execute() {
        handleDrive()
        handleIntake()
        handleShooter()

       if (Shooter.isAtSpeed && Shooter.targetSpeed.leftSpeeds != 0.RPM) Rumble.set(0.1,0.3, GenericHID.RumbleType.kRightRumble)

        Rumble.update()
    }
    private fun handleDrive(){
        var baseSpeed = if (OI.speedLower) C.SlowSpeed else C.DriveSpeed

        if (OI.reverseDrive) baseSpeed *= -1

        val leftSpeed  = baseSpeed * OI.leftThrottle
        val rightSpeed = baseSpeed * OI.rightThrottle

        Drivetrain.voltageDrive(leftSpeed * C.MaxVoltage, rightSpeed * C.MaxVoltage)
    }
    private fun handleIntake() = when {
        OI.feedToShoot -> Intake.runIntake(Constants.IntakeConstants.feedingSpeed)
        OI.intakeThrottle != 0.0 -> Intake.runIntake(OI.intakeThrottle.clamp(
                -Constants.IntakeConstants.reverseSpeed,
                Constants.IntakeConstants.pickupSpeed))
        else -> Intake.stop()
    }
    private fun handleShooter() = when {
        OI.shooterThrottle != 0.0 -> Shooter.runOpenLoop(OI.shooterThrottle)
        OI.shooterToSpeaker       -> Shooter.runClosedLoop(Constants.ShooterConstants.AmpSpeed)
        OI.shooterToAmp           -> Shooter.runClosedLoop(Constants.ShooterConstants.SpeakerSpeed)
        else -> Shooter.stop()
    }

    private fun handleClimb() = when(OI.climb) {
        OI.DirectionalPOV.UP   -> Climber.climb(Constants.ClimbConstants.ClimbPos.Extend)
        OI.DirectionalPOV.DOWN -> Climber.climb(Constants.ClimbConstants.ClimbPos.Retract)
        else -> Climber.stop()
    }


    object OI {
        val operatorController = XboxController(2)
        val commandOperatorController = CommandXboxController(2)
        val driverControllerL = Joystick(0) //TODO: Fix!
        val driverControllerR = Joystick(1)

        
        //New joystick tank drive code
        val leftThrottle  get() = driverControllerL.y.processInput(0.1,SquareMode.NORMAL,false)
        val rightThrottle get() = driverControllerR.y.processInput(0.1,SquareMode.NORMAL,false)

        val speedLower get() = driverControllerR.trigger
        val reverseDrive get() = driverControllerL.trigger
        val intakeThrottle get() = operatorController.leftY.processInput(readjust = false)
        val feedToShoot get() = operatorController.leftTriggerAxis.absGreaterThan(0.1)
        val shooterThrottle get() = operatorController.rightY.processInput(readjust = false).absoluteValue
        val shooterToAmp get() = operatorController.aButton
        val shooterToSpeaker get() = operatorController.yButton
        val climb get() = operatorController.pov.DirectionY()

        enum class SquareMode {
            NORMAL,
            SQUARED,
            CUBED
        }

        private fun Double.processInput(deadzone : Double = 0.1, squared : SquareMode = SquareMode.NORMAL, readjust : Boolean = true) : Double{
            var processed = this

            if (processed.within(deadzone)) return 0.0

            if(readjust) processed = ((processed.absoluteValue - deadzone)/(1 - deadzone))*processed.sign

            return when (squared) {
                SquareMode.SQUARED -> processed.pow(2) * this.sign
                SquareMode.CUBED   -> processed.pow(3)
                SquareMode.NORMAL  -> processed
            }
        }
        private fun Double.absGreaterThan(target: Double): Boolean{
            return this.absoluteValue > target
        }

        enum class DirectionalPOV(val degrees: Int){
            UP(0),
            RIGHT(90),
            DOWN(180),
            LEFT(270),
            NEUTRAL(-1)
        }
        fun Int.DirectionY() : DirectionalPOV{
            if(this == 45 || this == 0 || this == 315) return DirectionalPOV.UP
            if(this == 135 || this == 180 || this == 225) return DirectionalPOV.DOWN
            return DirectionalPOV.NEUTRAL
        }
    }
    object Rumble {
        private val rumbleTimer = Timer()
        private var rumbleTime = 0.0

        //Time in seconds
        fun set(time : Double, power : Double, side : GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble) {
            rumbleTime = time;
            OI.operatorController.setRumble(side, power)
            rumbleTimer.restart()
        }

        fun update() {
            if (rumbleTimer.hasElapsed(rumbleTime)) OI.operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        }
    }
}