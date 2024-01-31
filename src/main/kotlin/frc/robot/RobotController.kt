package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand

import frc.robot.commands.TeleOp
import frc.robot.subsystems.Drivetrain

/* Main code for controlling the robot. Mainly just links everything together.

    The hardware of the robot (what motor controllers, etc) is defined in RobotHardware.kt

    Driver control is defined in TeleOp.kt

*/

object RobotController : TimedRobot() {

    val autos: Map<String,Command> = mapOf(
        //TODO: Autos go here!
        //ie 
        //"Description of auto" to TaxiAuto
    )
    private var autoChooser : SendableChooser<Command> = SendableChooser()
    public var driveModeChooser : SendableChooser<TeleOp.DriveMode> = SendableChooser()

    private var selectedAuto : Command = InstantCommand({})

    init {
        autoChooser.setDefaultOption("No Auto",RunCommand({
            Drivetrain.percentDrive(0.0, 0.0)
        }) )

        for (auto in autos) {
            autoChooser.addOption(auto.key, auto.value);
        }

        SmartDashboard.putData(autoChooser)
        driveModeChooser.setDefaultOption("Default",TeleOp.DriveMode.DEFAULT)
        driveModeChooser.addOption("Child",TeleOp.DriveMode.CHILD)
        SmartDashboard.putData(driveModeChooser)

    }
    
    override fun robotInit() {
        //Initialize the robot!
        //CameraServer.startAutomaticCapture() //TODO: Can we offload camera streaming to a Raspberry Pi?

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
    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        selectedAuto.cancel() //Stop Auto before running teleop!
        TeleOp.schedule()
    }
    override fun teleopPeriodic() {}

    override fun simulationInit() {} //Runs only in simulation mode (other functions run regardless of whether robot is simulated or not)

    override fun disabledInit() {
        //Runs as soon as the robot is disabled, use to deactivate motors safely, etc
    }
    override fun disabledPeriodic() {
        //Runs only while robot is disabled- Use to hold motors in position for safety reasons.
        // Try to avoid putting code here- often unsafe.
    }

    override fun testInit() {}
    override fun testPeriodic() {}
}