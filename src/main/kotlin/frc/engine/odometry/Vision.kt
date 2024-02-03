package frc.engine.odometry
// Filed adapted from 2898s 2023 Charged Up code
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Odometry
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*
//TODO: Update for Photon
class Vision (
    CameraName: String
) {
    val cam = PhotonCamera(CameraName);
    var robotToCam = Transform3d(
        Translation3d(0.5, 0.0, 0.5),
        Rotation3d(0.0, 0.0, 0.0)
    )

    val aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)
    val PoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cam,
        robotToCam
    )
    fun getEstimatedPose(prevEstimatedRobotPose: Pose2d?): EstimatedRobotPose? {
        PoseEstimator.setReferencePose(prevEstimatedRobotPose)
        val pose = PoseEstimator.update() ?: return null
        return pose.get()

    }
}