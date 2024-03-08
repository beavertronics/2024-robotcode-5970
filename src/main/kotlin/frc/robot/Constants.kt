package frc.robot

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import frc.engine.controls.PIDConstants
import frc.engine.controls.SimpleMotorFeedForwardConstants

import frc.engine.utils.*

/* Constants for tuning the robot code.

This should NOT include hardware constants (ie CAN bus IDs, wheel diameter and gear ratios); 
things that are related to physical objects on the robot and are meant to be set once should 
be located in RobotHardware.kt instead.

*/

object Constants {
    object DriveConstants {
        // Yoinked from 2898 charged up code.
        // Ramsete parameters, see [https://file.tavsys.net/control/controls-engineering-in-frc.pdf] page 81
        // **DO NOT CHANGE THESE PARAMETERS**
        const val DRIVETRAIN_RAMSETE_B = 5.0  // Higher values make it more aggressively stick to the trajectory. 0 < B
        const val DRIVETRAIN_RAMSETE_Z = 0.7  // Higher values give it more dampening. 0 < Z < 1

        const val LeftMotorMainID = 21
        const val LeftMotorSubID  = 22
        const val RightMotorMainID = 23
        const val RightMotorSubID  = 24

        val MaxVelocity = `M/s`(2.0)
        val MaxAcceleration = MetersPerSecondSquared(0.5)

        const val MotorRevsPerWheelRev = (50/14) * (48/16) //~10:1 As taken from CAD- should verify after drivetrain is built in the real world
        val WheelDiameter = Inches(6.0).meterValue()
        val TrackWidth = Meters(0.0) //TODO Get track width

        val FF_CONSTANTS = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)
        val PID_CONSTANTS = PIDConstants(1.0,0.0,0.0)



        const val CurrentLimit = 30 //amps, per motor controller
        /* See
           https://the-charge.com/uploads/3/5/3/0/35304458/testing_and_analysis_of_first_robotics_batteries__2018_.pdf
           For info on battery characteristics
         */
    }
    object IntakeConstants {
        const val TopMotorID = 28
        const val BottomMotorID = 27

        const val CurrentLimit = 20 //Current limit 12 amps?
        const val limitSwitchChannel = 1

        const val pickupSpeed = 0.7 //TODO set intake speed.
        const val pushforwardSpeed = 0.3 //TODO set pullback speed. Multiplied by -1
        const val reverseSpeed = 0.5 //TODO set outtake Speed.
        const val feedingSpeed = 0.7 //TODO set feeding speed.

        const val feedingTime = 0.5 //TODO set feedingTime. In seconds
        const val unfeedTime = 0.5 //TODO set feedingTime. In seconds


    }
    object ShooterConstants {
        const val LeftFlywheelMotorID  = 25
        const val RightFlywheelMotorID = 26

        /** Proportional to the error (if it's bad, fix it. If it's really bad, fix it harder based on how bad it is)*/
        const val KP            = 1.0 // tune later

        /** Proportional to the slope (derivative) of the error
         * (If it's OK but it's starting to go bad, fix it ahead of time, and if it's bad but it's
         * getting close to being good, fix it less hard so it doesn't overshoot)*/
        const val KD            = 0.0 // tune later

        /** Propotional to the integral of the error
         * (If it's been bad for a long time, fix it harder.)
         * Usually very unstable; much safer to leave at 0.0 if possible
         */
        const val KI            = 0.0

        /** (roughly) how much voltage to overcome static friction */
        const val KS            = 0.0
        /** How much voltage to maintain a velocity*/
        const val KV            = 0.0
        /** How much voltage to accelerate- Can go unused (0) */
        const val KA            = 0.0

        const val CurrentLimit = 20 //amps, per motor controller
        val SpeakerPoly = Polynomial() //TODO Desmos this

        val SpeakerSpeed = 0.RPM //Todo
        val AmpSpeed = 0.RPM //Todo

    }
    object OdometryConstants {
        @Suppress("removal")
        val VisionDeviation = MatBuilder(Nat.N3(),Nat.N1()).fill(3.0,3.0,1000.0)
    }

    object TeleopConstants {
        const val MaxIntakeSpeed = 0.5
        const val MaxVoltage = 12.0
        const val DriveSpeed = 1.0 //TODO set drive speed
        const val SlowSpeed = 0.3 //TODO set speed boost speed
        const val QuickTurnDeadzone = 0.1 //TODO set quick turn dead zone
        const val QuickTurnSpeed = 0.7 //TODO set quick turn speed.

        //const val MaxSpeed = 5.0 //M/s
        //TODO: Maybe change to feet per second? Metric is Good, but many teams communicate drivetrain speed in feet per second so for communicating quickly it could be worth leaving in fps.
        //  Possible implementation:
        //    FeetPerSecond(16.5).metersPerSecondValue()
        //    (also import utils/Units.kt)
    }
    object ClimbConstants {

        enum class ClimbPos {
            Extend,
            Retract,
            Chill
        }

        const val RightMotorID = 29
        const val LeftMotorID = 30



        const val CurrentLimit = 30 //amps, per side. See drivetrain current limit.

        const val MotorRevsToRetract = (7/1) * (7/1) * 10.0 //TODO: DANGER! Rough estimate; based off of CAD but makes assumptions about winch cord stacking.
        //How many revolutions of the motor does it take to fully contract the lifter?

        const val ExtendVoltage = 10.0 //TODO: Tune!
        const val RetractVoltage = 10.0 //TODO: Tune!

        //Note: May require profiling to prevent slamming down on chain- test!

        //NOTE: Feedforwards control would normally be required in order to fight gravity, but the climber is actually spring loaded to be in the extended position
    }


}