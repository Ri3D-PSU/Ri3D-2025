package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
  private final String CAMERA_NAME = "Camera_Module_v1";
  private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Transform3d ROBOTTOCAM =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5),
          new Rotation3d(
              0, 0,
              0)); // From docs, cam mounted facing forward, half a meter forward of center, half a
  // meter up from center.

  private PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
  private PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          APRILTAGFIELDLAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOTTOCAM);

  private NetworkTable cameraTable =
      NetworkTableInstance.getDefault().getTable(String.format("photonvision/%s", CAMERA_NAME));
  private DoubleArraySubscriber poseSub =
      cameraTable.getDoubleArrayTopic("targetPose").subscribe(new double[] {});

  public AprilTagVisionIOPhotonvision() {}

  public void updateInputs(
      LoggableAprilTagVisionIOInputs loggableInputs,
      UnloggableAprilTagVisionIOInputs unloggableInputs) {
    loggableInputs.ntPose = poseSub.get();
    unloggableInputs.unreadResults = camera.getAllUnreadResults();
    unloggableInputs.latestEstimatedPose = updatePoseEstimator(unloggableInputs.unreadResults);
    loggableInputs.latestCamToTagTranslation = getCamToTag(unloggableInputs.unreadResults);
  }

  public Optional<EstimatedRobotPose> updatePoseEstimator(List<PhotonPipelineResult> results) {
    Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> updatedPose = poseEstimator.update(result);
      latestEstimatedPose = !updatedPose.isEmpty() ? updatedPose : latestEstimatedPose;
    }
    return latestEstimatedPose;
  }

  public Transform3d getCamToTag(List<PhotonPipelineResult> results) {
    Transform3d camToTag = new Transform3d();
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        camToTag = result.getBestTarget().getBestCameraToTarget();
      }
    }
    return camToTag;
  }
}
