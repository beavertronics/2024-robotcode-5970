package frc.robot

import frc.engine.utils.*

/* Important Constants related to Robot Safety (Do not change these values unless you know what you are doing)

All other constants should be located nearby to where they are used (ie in subsystem file)

*/

object Safety {
    object DriveConstants {
        val MaxVelocity = `M/s`(2.0)
        val MaxAcceleration = MetersPerSecondSquared(0.5)

        const val CURRENT_LIMIT = 30 //amps, per motor controller on the drivetrain
        // See https://the-charge.com/uploads/3/5/3/0/35304458/testing_and_analysis_of_first_robotics_batteries__2018_.pdf
        // for more info on battery characteristics
    }
    object IntakeConstants {
        const val CURRENT_LIMIT = 20 //Current limit 12 amps?
    }
    object ShooterConstants {
        const val CURRENT_LIMIT = 20 //amps, per motor controller
    }
    object TeleopConstants {
        const val MAX_VOLTAGE = 12.0
        const val DRIVE_SPEED = 1.0 //TODO set drive speed
        const val SLOW_SPEED = 0.3 //TODO set speed boost speed

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