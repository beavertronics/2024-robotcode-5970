package frc.robot.commands

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.engine.utils.Sugar.within

import frc.robot.Constants.TeleopConstants as C
import kotlin.math.*

import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    var shooting = false
    val shootTimer = Timer()

    override fun initialize() {
        addRequirements(Drivetrain,Intake,Shooter)
        
    }

    override fun execute() {
        OI.Rumble.update()
        // HANDLE DRIVE
        var baseSpeed = if (OI.speedBoost) C.speedBoostSpeed else C.driveSpeed

        if (OI.reverseDrive) baseSpeed *= -1
        val leftSpeed  = baseSpeed * OI.leftThrottle
        val rightSpeed = baseSpeed * OI.rightThrottle

        Drivetrain.rawDrive(leftSpeed * C.MaxVoltage, rightSpeed * C.MaxVoltage)

        // HANDLE INTAKE
        when{
            OI.shoot -> Intake.feed().schedule()
            OI.manualIntakeSpeed > 0.3 -> { // OUTTAKE
                Intake.outtakeSpeed = OI.manualIntakeSpeed * C.MaxIntakeSpeed
                if(Intake.currentCommand.name != "FEEDING") Intake.outtake().schedule()
            }
            OI.manualIntakeSpeed < 0 -> { // INTAKE
                Intake.intakeSpeed = -OI.manualIntakeSpeed * C.MaxIntakeSpeed
                if(Intake.currentCommand.name == "IDLE" || Intake.currentCommand.name == "OUTTAKE" && Intake.currentCommand.name != "PICKUP") Intake.doIntake().schedule()
            }
            else -> if(Intake.currentCommand.name != "PULL_BACK" && Intake.currentCommand.name != "IDLE") Intake.idle().schedule() // IDLE

        }
        Intake.doIntake()
        Shooter.setSpeedRaw(OI.shooterSpeed)
    }

    object OI {

        private val commandOperatorController = CommandXboxController(0)
        private val operatorController = XboxController(0)
        private val driverControllerL = Joystick(1) //TODO: Fix!
        private val driverControllerR = Joystick(2)
        init {
            //commandOperatorController.b().whileTrue(Intake.doIntake()) //WhileTrue does not repeat trying to intake once intaking finishes, but will stop if the button is let go.
            //commandOperatorController.rightBumper().whileTrue(Intake.feed())
        }
        //Driver Controls
        val leftThrottle  get() = driverControllerL.y.processInput(squared=SquareMode.SQUARED, readjust = false)
        val rightThrottle get() = driverControllerR.y.processInput(squared=SquareMode.SQUARED, readjust = false)

        val speedBoost get() = driverControllerR.trigger
        val reverseDrive get() = driverControllerL.trigger
        //Operator controls

        val shooterSpeed get() =  abs(operatorController.leftY.processInput())
        val manualIntakeSpeed get() = operatorController.rightY.processInput()
        val shoot get() = operatorController.rightTriggerAxis.absGreaterThan(0.5)



        enum class SquareMode {
            NORMAL,
            SQUARED,
            CUBED
        }

        /**
         * Processes user input to ensure smooth control of subsystems
         * @param deadzone returns 0 if the absolute value of the input is less than deadzone
         * @param squared whether to square the input (Automatically adjusts so that the sign of the output is not changed)
         * @param readjust readjusts the output so that deadzone is the starting point for the output (ex. if input = deadzone+0.1, output = 0.1)
         */
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
        object Rumble {
            private val rumbleTimer = Timer()
            private var rumbleTime  = 0.0

            fun set(time: Double, power: Double, side: GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble){
                rumbleTimer.reset()
                rumbleTime = time
                operatorController.setRumble(side, power)
                rumbleTimer.start()
            }
            fun update(){
                if(rumbleTimer.hasElapsed(rumbleTime)){
                    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                }
            }
        }

    }
}