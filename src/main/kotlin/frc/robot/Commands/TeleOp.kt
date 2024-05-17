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
import frc.engine.utils.RotationsPerSecond
import frc.engine.utils.Sugar.clamp
import frc.engine.utils.Sugar.within
import frc.robot.Safety

import frc.robot.Safety.TeleopConstants as C
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
        //handleClimb()


        Rumble.update()
    }
    private fun handleDrive(){
        var baseSpeed = if (OI.speedLower) C.SLOW_SPEED else C.DRIVE_SPEED

        if (OI.reverseDrive) baseSpeed *= -1
        //val avrgThrottle = (OI.leftThrottle + OI.rightThrottle)/2

        var leftSpeed  = baseSpeed * OI.leftThrottle//avrgThrottle
        var rightSpeed = baseSpeed * OI.rightThrottle//avrgThrottle

        /*val turn = ((OI.leftThrottleRaw - OI.rightThrottleRaw).absoluteValue)/2
        println(turn)

        if(OI.leftThrottleRaw < OI.rightThrottleRaw) {
            leftSpeed -= turn
            rightSpeed += turn
        }
        else {
            rightSpeed -= turn
            rightSpeed += turn
        }
        leftSpeed = leftSpeed.clamp(-1.0,1.0)
        rightSpeed = rightSpeed.clamp(-1.0,1.0)*/

        if(!OI.reverseDrive) Drivetrain.voltageDrive(leftSpeed * C.MAX_VOLTAGE, rightSpeed * C.MAX_VOLTAGE)
        else Drivetrain.voltageDrive(rightSpeed * C.MAX_VOLTAGE, leftSpeed * C.MAX_VOLTAGE)
    }
    private fun handleIntake() = when {
        OI.feedToShoot -> Intake.runIntake(Intake.FEEDING_SPEED)
        OI.intakeThrottle < 0.0 -> Intake.runIntake(OI.intakeThrottle.clamp(
                -Intake.REVERSE_SPEED,
            Intake.PICKUP_SPEED))
        OI.intakeThrottle > 0.0 -> {
            val intakeSpeed = OI.intakeThrottle.clamp(
                    -Intake.REVERSE_SPEED,
                    Intake.PICKUP_SPEED)
            Intake.runIntake(intakeSpeed/2, intakeSpeed)
        }
        else -> Intake.stop()
    }
    private fun handleShooter() = when {
        OI.shooterThrottle != 0.0 -> {
            if (Shooter.openLoopIsAtSpeed()) {Rumble.set(0.1,0.3, GenericHID.RumbleType.kRightRumble)}
            Shooter.runOpenLoop(OI.shooterThrottle)
        }
        OI.shooterToSpeaker       -> {
            if (Shooter.isAtSpeed && Shooter.targetSpeed.leftSpeeds != 0.RotationsPerSecond) Rumble.set(0.1,0.3, GenericHID.RumbleType.kRightRumble)
            Shooter.runClosedLoop(Shooter.SPEAKER_SPEED)
        }//Shooter.runClosedLoop(Shooter.leftTestAmpSpeed,Shooter.rightTestAmpSpeed)
        OI.shooterToAmp           -> {
            if (Shooter.isAtSpeed && Shooter.targetSpeed.leftSpeeds != 0.RotationsPerSecond) Rumble.set(0.1,0.3, GenericHID.RumbleType.kRightRumble)
            Shooter.runClosedLoop(Shooter.AMP_SPEED)
        }
        else -> Shooter.stop()
    }

    private fun handleClimb() = when(OI.climb) {
        OI.DirectionalPOV.UP   -> Climber.climb(Safety.ClimbConstants.ClimbPos.Extend)
        OI.DirectionalPOV.DOWN -> Climber.climb(Safety.ClimbConstants.ClimbPos.Retract)
        else -> Climber.stop()
    }


    object OI {
        val operatorController = XboxController(2)
        //val commandOperatorController = CommandXboxController(2)
        val driverControllerL = Joystick(0) //TODO: Fix!
        val driverControllerR = Joystick(1)

        
        //New joystick tank drive code
        val leftThrottleRaw  get() = driverControllerL.y
        val rightThrottleRaw  get() = driverControllerL.y

        val leftThrottle  get() = driverControllerL.y.processInput(0.08,SquareMode.NORMAL,true)
        val rightThrottle get() = driverControllerR.y.processInput(0.08,SquareMode.NORMAL,true)

        val speedLower get() = driverControllerR.trigger
        val reverseDrive get() = driverControllerL.trigger
        val intakeThrottle get() = operatorController.leftY.processInput(readjust = false)
        val feedToShoot get() = operatorController.rightTriggerAxis.absGreaterThan(0.1)
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