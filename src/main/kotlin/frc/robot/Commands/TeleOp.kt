package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.engine.utils.Sugar.within

import frc.robot.Constants.TeleopConstants as C
import frc.robot.Constants.IntakeConstants
import kotlin.math.*

import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    override fun initialize() {
        addRequirements(Drivetrain,Intake,Shooter)
        
    }

    override fun execute() {
    
        var baseSpeed = if (OI.speedBoost) C.speedBoostSpeed else C.driveSpeed

        if (OI.reverseDrive) baseSpeed *= -1

        val leftSpeed  = baseSpeed * OI.leftThrottle
        val rightSpeed = baseSpeed * OI.rightThrottle

        Drivetrain.rawDrive(leftSpeed * C.MaxVoltage, rightSpeed * C.MaxVoltage)

          
        Shooter.setSpeedRaw(OI.shooterSpeed)

        /*if (OI.manualIntakeSpeed != 0.0) {
            Intake.run {}
            Intake.runIntake(OI.manualIntakeSpeed);
        }*/

        if (!Intake.limitSwitch.get()) {
            OI.Rumble.set(0.25,1.0)
        }

        OI.Rumble.update()

    }

    object OI {
        private val operatorController = CommandXboxController(0)
        private val driverControllerL = Joystick(1) //TODO: Fix!
        private val driverControllerR = Joystick(2)

        
        //New joystick tank drive code
        public val leftThrottle  get() = driverControllerL.getY().processInput(0.1,SquareMode.SQUARED,false)
        public val rightThrottle get() = driverControllerR.getY().processInput(0.1,SquareMode.SQUARED,false)

        public val speedBoost get() = driverControllerR.trigger
        public val reverseDrive get() = driverControllerL.trigger
        val shooterSpeed get() =  abs(operatorController.leftY).processInput()
        val manualIntakeSpeed get() = operatorController.rightY.processInput(deadzone = 0.1, readjust = false)
        val getManualIntakeSpeed: () -> Double = { -manualIntakeSpeed }

        init {

            operatorController
                .axisGreaterThan(XboxController.Axis.kRightTrigger.value,0.1)
                .onTrue(Intake.feed())
            operatorController
                .axisGreaterThan(XboxController.Axis.kRightY.value,0.1)
                .onTrue(Intake.outtake(getManualIntakeSpeed))
            operatorController
                .axisLessThan(XboxController.Axis.kRightY.value,-0.1)
                .onTrue(Intake.doIntake(getManualIntakeSpeed))
                .onFalse(Intake.idle())
            //operatorController.b().onTrue(Intake.doIntake()) //WhileTrue does not repeat trying to intake once intaking finishes, but will stop if the button is let go.
            //operatorController.rightBumper().whileTrue(Intake.feed())
        }

        object Rumble {
            private val rumbleTimer = Timer()
            private var rumbleTime = 0.0

            private val xboxHID = operatorController.hid

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
}