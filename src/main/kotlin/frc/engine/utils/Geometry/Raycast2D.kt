package frc.engine.utils.geometry
// File adapted from 2898 2023 bpsrobotics engine
import frc.engine.utils.Sugar.eqEpsilon
import edu.wpi.first.math.geometry.Pose2d
import frc.engine.utils.Units.Angular.AngleUnit
import frc.engine.utils.Units.Angular.beaverRadians
import frc.engine.utils.Units.Angular.radians
import frc.engine.utils.Units.Linear.DistanceUnit
import frc.engine.utils.Units.Linear.meters
import kotlin.math.*

class Raycast2D(val origin : Vector2, val angle : AngleUnit){
    constructor(pose: Pose2d) : this(Vector2(pose),pose.rotation.beaverRadians)

}