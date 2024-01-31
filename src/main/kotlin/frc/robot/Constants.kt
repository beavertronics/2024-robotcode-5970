package frc.robot

import frc.engine.utils.Inches
import frc.engine.utils.Meters

/* Constants for tuning the robot code.

This should NOT include hardware constants (ie CAN bus IDs, wheel diameter and gear ratios); 
things that are related to physical objects on the robot and are meant to be set once should 
be located in RobotHardware.kt instead.

*/

object Constants {
    object DriveConstants {

        const val MotorRMainID   = 21 //TODO Get neos from electrical
        const val MotorRSubID   = 22 // Should start from 21, as is tradition.
        const val MotorLMainID    = 23
        const val MotorLSubID    = 24
    
        const val MotorRevsPerWheelRev = (50/14) * (48/16) //~10:1 As taken from CAD- should verify after drivetrain is built in the real world
        val WheelDiameter = Inches(6.0).meterValue()
        val TrackWidth = Meters(0.0) //TODO Get track width

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
        const val CurrentLimit = 30 //amps, per motor controller
        /* See
           https://the-charge.com/uploads/3/5/3/0/35304458/testing_and_analysis_of_first_robotics_batteries__2018_.pdf
           For info on battery characteristics
         */
    }

    object TeleopConstants {
        const val MaxSpeed = 5.0 //M/s
        //TODO: Maybe change to feet per second? Metric is Good, but many teams communicate drivetrain speed in feet per second so for communicating quickly it could be worth leaving in fps.
        //  Possible implementation:
        //    FeetPerSecond(16.5).metersPerSecondValue()
        //    (also import utils/Units.kt)
    }
}