package frc.robot.Commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.engine.utils.RPM
import frc.engine.utils.Sugar.within

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

    override fun execute() {
    
        var baseSpeed = if (OI.speedLower) C.SlowSpeed else C.DriveSpeed

        if (OI.reverseDrive) baseSpeed *= -1

        val leftSpeed  = baseSpeed * OI.leftThrottle
        val rightSpeed = baseSpeed * OI.rightThrottle

        //println("Teleop Execute")
        Drivetrain.voltageDrive(leftSpeed * C.MaxVoltage, rightSpeed * C.MaxVoltage)

       if (Shooter.isAtSpeed && Shooter.targetSpeed.leftSpeeds != 0.RPM) Rumble.set(0.1,0.3, GenericHID.RumbleType.kRightRumble)

        Rumble.update()
    }

    object OI {
        val operatorController = CommandXboxController(2)
        val driverControllerL = Joystick(0) //TODO: Fix!
        val driverControllerR = Joystick(1)

        
        //New joystick tank drive code
        val leftThrottle  get() = driverControllerL.y.processInput(0.1,SquareMode.NORMAL,false)
        val rightThrottle get() = driverControllerR.y.processInput(0.1,SquareMode.NORMAL,false)

        val speedLower get() = driverControllerR.trigger
        val reverseDrive get() = driverControllerL.trigger
        val manualShooterSpeed get() =  abs(operatorController.leftY).processInput()

        private val   feedToShoot : Trigger = operatorController.leftBumper()
        private val        pickup : Trigger = operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
        private val         eject : Trigger = operatorController.axisLessThan   (XboxController.Axis.kLeftY.value, -0.1)
        private val spinupShooter : Trigger = operatorController.axisGreaterThan(XboxController.Axis.kRightY.value,0.1)
                                                        .or(operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.1))
        private val spinupToSpeaker : Trigger = operatorController.y()
        private val     spinupToAmp : Trigger = operatorController.a()
        private val  retractClimber : Trigger = operatorController.povDown()
        private val   extendClimber : Trigger = operatorController.povUp()




        init {


            /* Left stick y = Digital intake control (-70 or 70 percent). If left bumper is not pressed, will stop after limit switch has been unpressed
            pickup.and(feedToShoot.negate())
                .whileTrue(Intake.doIntake()) //Don't repeat; will stop when limit switch is done being triggered
            */
            eject.whileTrue(Intake.doEject().repeatedly()) //Yes repeat; keep reversing note as long as driver says so*/

            // Left bumper = Shoot control- Allows shoot when pressed
            pickup.whileTrue(Intake.doFeed().repeatedly()) //Yes feed repeatedly

            // Right stick y = manual shooter control
            spinupShooter
                .whileTrue(Shooter.run {Shooter.runOpenLoop(operatorController.rightY.absoluteValue)} )

            // Y = shoot for speaker
            spinupToSpeaker
                .whileTrue(Shooter.doSpinupToSpeaker())
            // A = run shooter at amp speeds
            spinupToAmp
                .whileTrue(Shooter.doSpinupToAmp())
            /*// DPad up = Climber extend
            extendClimber
                .whileTrue(Climber.doExtend()t
            // DPad up = Climber retract
            retractClimber
                .whileTrue(Climber.doRetract())*/
        }

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
        private fun Double.abs_GreaterThan(target: Double): Boolean{
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

        private val xboxHID = OI.operatorController.hid

        //Time in seconds
        fun set(time : Double, power : Double, side : GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble) {
            rumbleTime = time;
            xboxHID.setRumble(side, power)
            rumbleTimer.reset();
            rumbleTimer.start();
        }

        fun update() {
            if (rumbleTimer.hasElapsed(rumbleTime)) {
                xboxHID.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }
        }
    }
}