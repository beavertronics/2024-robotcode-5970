package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Commands.Autos.OHGODTHEYGAVEUS2MINUTESTOTESTATCOMP_auto
import frc.robot.Commands.Autos.`Preload+BottomNote`
import frc.robot.Commands.Autos.`Preload+Mobility`
import frc.robot.Commands.SysID.SysID
import frc.robot.Commands.SysID.SysIDShooter
import frc.robot.Commands.SysID.drivetrainSys
import frc.robot.Commands.SysID.shooterSys
import frc.robot.Commands.TeleOp

import frc.robot.subsystems.Drivetrain

/* Main code for controlling the robot. Mainly just links everything together.

    The hardware of the robot (what motor controllers, etc) is defined in RobotHardware.kt

    Driver control is defined in TeleOp.kt

*/

object RobotController : TimedRobot() {

    val noAuto = RunCommand({Drivetrain.voltageDrive(0.0, 0.0)})
    var autos: Map<String,Command> = mapOf(
        //TODO: Autos go here!
        //ie 
        //"Description of auto" to TaxiAuto
        "Timed Mobility" to OHGODTHEYGAVEUS2MINUTESTOTESTATCOMP_auto(),
        "Bottom_Preload+Mobility" to `Preload+Mobility`(),
        "Bottom_Preload+BottomNote" to `Preload+BottomNote`()
    )
    var tests: Map<String,Command> = mapOf(
            //TODO: Tests go here!
            //ie
            //"Description of test" to TaxiTest
            "Shooter SysID" to SysIDShooter(),
            "Drivetrain SysID" to SysID(drivetrainSys)
    )

    private var autoChooser : SendableChooser<Command> = SendableChooser()
    private var selectedAuto : Command = noAuto
    init {
        autoChooser.setDefaultOption("No Auto", noAuto)
        for (auto in autos) {
            autoChooser.addOption(auto.key, auto.value);
        }
        for (test in tests) {
            autoChooser.addOption(test.key, test.value);
        }

        SmartDashboard.putData(autoChooser)
    }
    
    override fun robotInit() {
        //Initialize the robot!
        CameraServer.startAutomaticCapture() //TODO: Can we offload camera streaming to a Raspberry Pi? Should we?

    }
    override fun robotPeriodic() {
        //Runs while the robot is on, regarless of whether it is enabled.
        // (use for telemetry, command scheduler)
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        selectedAuto = autoChooser.selected;
        selectedAuto.schedule();
    }
    override fun autonomousPeriodic() {} //TODO: Unnecesary with command-based programming?

    override fun teleopInit() {
        selectedAuto.cancel() //Stop Auto before running teleop
        TeleOp.schedule()
    }
    override fun teleopPeriodic() {} //TODO: Unnecessary with command-based programming?

    override fun simulationInit() {} //Runs only in simulation mode (other functions run regardless of whether robot is simulated or not)

    override fun disabledInit() {
        //Runs as soon as the robot is disabled, use to deactivate motors safely, etc
        TeleOp.Rumble.set(0.0, 0.0)
    }
    override fun disabledPeriodic() {
        //Runs only while robot is disabled- Use to hold motors in position for safety reasons.
        // Try to avoid putting code here- often unsafe.
    }


    private var testChooser : SendableChooser<Command> = SendableChooser()
    private var selectedTest : Command = noAuto
    init {
        //testChooser.setDefaultOption("No Auto", noAuto)


        //SmartDashboard.putData(testChooser)
    }

    override fun testInit() {
        //val commandOperatorController = CommandXboxController(2)
        //commandOperatorController.a().whileTrue(selectedTest)
        //TeleOp.OI.commandOperatorController.a().whileTrue(selectedTest)
        selectedTest.schedule()
    }
    override fun testPeriodic() {
        //Yay!
    }
}