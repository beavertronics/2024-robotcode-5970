package frc.robot.subsystems
//File adapted from 2898's 2023 Charged Up Code
import frc.engine.odometry.PoseProvider
import frc.engine.utils.Degrees
import frc.engine.utils.Meters
import frc.engine.utils.MetersPerSecond
import com.kauailabs.navx.frc.AHRS as NAVX
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsytems.Drivetrain
import frc.robot.Constants.DriveConstants as D


object Odometry : SubsystemBase(), PoseProvider {

    var navx = NAVX()

    private val otherProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())
    val leftVel get() =  MetersPerSecond(Drivetrain.leftEncoder.velocity)
    val rightVel get() = MetersPerSecond(Drivetrain.rightEncoder.velocity)
    val vels get() = DifferentialDriveWheelSpeeds(leftVel.metersPerSecondValue(), rightVel.metersPerSecondValue())
    private val thirdProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())

    override var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set

    val field = Field2d()
    val initial = Pose2d(11.789039, 0.74, Rotation2d.fromDegrees(0.0))

    init {
        val stdDevs = Matrix(Nat.N3(), Nat.N1())
        Vision.listeners.add { visionPose, stdDevArray, time ->
            if (visionPose.translation.getDistance(pose.translation) > 2.0) return@add
//            for (i in stdDevArray.indices) {
//                stdDevArray[i] *= 1.5
//            }
            stdDevArray.copyInto(stdDevs.data, 0, 0, 3)
            otherProvider.setVisionMeasurementStdDevs(stdDevs)
            otherProvider.addVisionMeasurement(Pose2d(visionPose.translation, thirdProvider.estimatedPosition.rotation), time)
        }
        reset(initial)
    }

    fun zero() {
//        val initial = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))
        reset(initial)
    }

    override fun periodic() {
        update()
    }

    override fun reset(x: Meters, y: Meters, theta: Degrees) {
        val p = Pose2d(x.value, y.value, Rotation2d.fromDegrees(theta.value))
        otherProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
        thirdProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
    }

    override fun update() {
        pose = otherProvider.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)
        thirdProvider.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)

        field.robotPose = pose
        field.getObject("pure odometry").pose = thirdProvider.estimatedPosition
        SmartDashboard.putData(field)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}