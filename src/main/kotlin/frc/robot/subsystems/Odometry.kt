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
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Drivetrain
import frc.engine.odometry.Vision
import frc.robot.Constants
import frc.robot.Constants.DriveConstants as D


object Odometry : SubsystemBase(), PoseProvider {


    var navx = NAVX()
    private val vision = Vision("testCamera")

    private val visionProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())
    private val encoderOnly = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())

    val leftVel get() =  MetersPerSecond(Drivetrain.leftEncoder.velocity)
    val rightVel get() = MetersPerSecond(Drivetrain.rightEncoder.velocity)
    val vels get() = DifferentialDriveWheelSpeeds(leftVel.metersPerSecondValue(), rightVel.metersPerSecondValue())
    val chassisSpeeds get() = ChassisSpeeds(leftVel.metersPerSecondValue(), rightVel.metersPerSecondValue(), navx.rate)
    val getChassesSpeeds: () -> ChassisSpeeds = { chassisSpeeds }
    val getCurrentSpeeds: () -> DifferentialDriveWheelSpeeds = { vels }
    private val thirdProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(D.TrackWidth.meterValue()), navx.rotation2d, 0.0, 0.0, Pose2d())

    override var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set
    val getPose: () -> Pose2d = { pose }

    val field = Field2d()
    //val initial = Pose2d(11.789039, 0.74, Rotation2d.fromDegrees(0.0))

    init {

    }

    //fun zero() {
    //    val initial = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))
    //    reset(initial)
    //}


    override fun reset(x: Meters, y: Meters, theta: Degrees) {
        val p = Pose2d(x.value, y.value, Rotation2d.fromDegrees(theta.value))
        visionProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
        encoderOnly.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
    }

    /*override fun periodic() {
        update()
    }*/

    override fun periodic() {
        val visionMeasurements = vision.getEstimatedPose(pose)
        if(visionMeasurements != null){
            visionProvider.setVisionMeasurementStdDevs(Constants.OdometryConstants.VisionDeviation)
            visionProvider.addVisionMeasurement(visionMeasurements.estimatedPose.toPose2d(), visionMeasurements.timestampSeconds)
        }

        pose = visionProvider.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)
        encoderOnly.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)

        field.robotPose = pose
        field.getObject("pure odometry").pose = encoderOnly.estimatedPosition
        SmartDashboard.putData(field)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}