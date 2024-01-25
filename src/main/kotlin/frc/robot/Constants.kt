package frc.robot

import frc.engine.utils.MetersPerSecond

object Constants {
    object DriveConstants {
        const val MotorRMainID   = 1 //TODO Get neos from electrical
        const val MotorRSubID   = 2 // Should start from 21, as is tradition.
        const val MotorLMainID    = 3
        const val MotorLSubID    = 4
        const val KP            = 1.0 // tune later
        const val KD            = 0.0 // tune later
        /** How much voltage to overcome friction */
        const val KS            = 0.0
        /** How much voltage to maintain a velocity */
        const val KV            = 0.0
        /** How much voltage to accelerate */
        const val KA            = 0.0
    }
    object TeleopConstants {
        const val MaxSpeed = 5.0 //M/s
    }
}