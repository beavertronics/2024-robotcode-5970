package frc.robot

import frc.engine.utils.MetersPerSecond

object Constants {
    object DrivetrainConstants {
        const val RightNEO1ID   = 1 //TODO Get neos from electrical
        const val RightNEO2ID   = 2
        const val LeftNEO1ID    = 3
        const val LeftNEO2ID    = 4
        /** Proportional to the error */
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