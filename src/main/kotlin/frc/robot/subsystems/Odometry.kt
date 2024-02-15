package frc.robot.subsystems
//File adapted from 2898's 2023 Charged Up Code
import frc.engine.odometry.PoseProvider
import frc.engine.utils.Degrees
import frc.engine.utils.Meters
import frc.engine.utils.MetersPerSecond
import com.kauailabs.navx.frc.AHRS as NAVX
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
import frc.robot.subsystems.Drivetrain
import frc.engine.odometry.Vision
import frc.engine.odometry.testLayout1
import frc.robot.Constants
import frc.robot.Constants.DriveConstants as D

object Odometry : SubsystemBase(), PoseProvider {

    var navx = NAVX()
    private val vision = Vision("Camera1", testLayout1)

    private val visionProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())
    private val encoderOnly = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())

    val leftVel get() =  0.0
    val rightVel get() = 0.0
    val vels get() = DifferentialDriveWheelSpeeds(leftVel, rightVel)

    override var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set

    val field = Field2d()
    val initial = Pose2d(11.789039, 0.74, Rotation2d.fromDegrees(0.0))

    init {

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
        visionProvider.resetPosition(navx.rotation2d, 0.0, 0.0, p)
        encoderOnly.resetPosition(navx.rotation2d, 0.0, 0.0, p)
    }

    override fun update() {
        val visionMeasurements = vision.getEstimatedPose(pose)
        if(visionMeasurements != null){
            visionProvider.setVisionMeasurementStdDevs(vision.getStdDev())
            visionProvider.addVisionMeasurement(visionMeasurements.estimatedPose.toPose2d(), visionMeasurements.timestampSeconds)
        }

        pose = visionProvider.update(navx.rotation2d, 0.0, 0.0)
        encoderOnly.update(navx.rotation2d, 0.0, 0.0)

        field.robotPose = pose
        field.getObject("pure odometry").pose = encoderOnly.estimatedPosition
        SmartDashboard.putNumberArray("Odometry/Pose", doubleArrayOf(pose.x,pose.y))
        SmartDashboard.putData(field)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}
