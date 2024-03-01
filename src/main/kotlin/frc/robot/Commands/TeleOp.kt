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
    
        var baseSpeed = if (OI.speedLower) C.slowSpeed else C.driveSpeed

        if (OI.reverseDrive) baseSpeed *= -1

        val leftSpeed  = baseSpeed * OI.leftThrottle
        val rightSpeed = baseSpeed * OI.rightThrottle

        Drivetrain.rawDrive(leftSpeed * C.MaxVoltage, rightSpeed * C.MaxVoltage)

        /*if (OI.manualIntakeSpeed != 0.0) {
            Intake.run {}
            Intake.runIntake(OI.manualIntakeSpeed*IntakeConstants.pickupSpeed );
        }*/

        if (!Intake.limitSwitch.get()) Rumble.set(0.25,1.0, GenericHID.RumbleType.kRightRumble)
        if (Shooter.isAtSpeed) Rumble.set(0.1,0.5, GenericHID.RumbleType.kLeftRumble)

        Rumble.update()

    }

    object OI {
        val operatorController = CommandXboxController(2)
        val driverControllerL = Joystick(0) //TODO: Fix!
        val driverControllerR = Joystick(1)

        
        //New joystick tank drive code
        public val leftThrottle  get() = driverControllerL.y.processInput(0.1,SquareMode.SQUARED,false)
        public val rightThrottle get() = driverControllerR.y.processInput(0.1,SquareMode.SQUARED,false)

        public val speedLower get() = !driverControllerR.trigger
        public val reverseDrive get() = driverControllerL.trigger
        /* 
        val manualIntakeSpeed get() = operatorController.rightY.processInput(deadzone = 0.1, readjust = false)
        private val getManualIntakeSpeed: () -> Double = { -manualIntakeSpeed }
        */
        val manualShooterSpeed get() =  abs(operatorController.leftY).processInput()
        //private val getManualShooterSpeed: () -> Double = { manualShooterSpeed }

        val feedToShoot : Trigger = operatorController.leftBumper()
        val      pickup : Trigger = operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
        val       eject : Trigger = operatorController.axisLessThan   (XboxController.Axis.kLeftY.value, -0.1)


        init {
            /*
            Left stick y = Digital intake control (-70 or 70 percent). If left bumper is not pressed, will stop after limit switch has been unpressed

            Left bumper = Shoot control- Allows shoot when pressed
            */
            pickup.and(feedToShoot.negate())
                .whileTrue(Intake.doIntake()) //Don't repeat; will stop when limit switch is done being triggered

            eject.and(feedToShoot.negate())
                .whileTrue(Intake.doEject().repeatedly()) //Yes repeat; keep reversing note as long as driver says so
            
            feedToShoot.whileTrue(Intake.doFeed().repeatedly()) //Yes feed repeatedly

            /*
            Y = run shooter at speaker speeds
            A = run shooter at amp speeds
            
            Right stick y = manually control shooter, absolute value
            */

            // Right stick y = manual shooter control
            operatorController
                .axisGreaterThan(XboxController.Axis.kRightY.value,0.1)
                    .or(operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.1))
                .whileTrue(Shooter.run {Shooter.runOpenLoop(operatorController.rightY.absoluteValue)} )//Looks like shooter speed is only passed once, but manualShooterSpeed is actually a callback so it's fine :/

            // Y = shoot for speaker
            operatorController
                .y()
                .whileTrue(Shooter.doSpinupToSpeaker())
            // Shooting Amp Speeds
            operatorController
                .a()
                .whileTrue(Shooter.doSpinupToAmp())

            operatorController.povDown()
                .whileTrue(Climber.doRetract())
                
            operatorController.povUp()
                .whileTrue(Climber.doExtend())

            //operatorController.b().onTrue(Intake.doIntake()) //WhileTrue does not repeat trying to intake once intaking finishes, but will stop if the button is let go.
            //operatorController.rightBumper().whileTrue(Intake.feed())
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