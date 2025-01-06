package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
  private final String CAMERA_NAME = "Camera_Module_v1";
  private LoggableAprilTagVisionIOInputs loggableInputs = new LoggableAprilTagVisionIOInputs();
  PIDController rotatePid = new PIDController(0.125, 0, 0);
  PIDController xPid = new PIDController(1, 0, 0.005);
  PIDController yPid = new PIDController(0.0635, 0, 0.0055);
  private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Transform3d ROBOTTOCAM =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5),
          new Rotation3d(
              0, 0,
              0)); // From docs, cam mounted facing forward, half a meter forward of center, half a
  // meter up from center.

  PhotonCamera camera = new PhotonCamera("photonvision/Camera_Module_v1");
  private PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          APRILTAGFIELDLAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOTTOCAM);
  double[] def1 = {0, 0, 0, 0, 0, 0};
  private NetworkTable cameraTable =
      NetworkTableInstance.getDefault().getTable(String.format("photonvision/%s", CAMERA_NAME));
  private DoubleArraySubscriber poseSub =
      cameraTable.getDoubleArrayTopic("targetPose").subscribe(def1);
  double defVal = 0;
  private DoubleSubscriber yaw = cameraTable.getDoubleTopic("targetYaw").subscribe(defVal);
  private DoubleSubscriber x = cameraTable.getDoubleTopic("targetPixelsX").subscribe(defVal);
  private DoubleSubscriber y = cameraTable.getDoubleTopic("targetArea").subscribe(defVal);
  private BooleanSubscriber hasTarget = cameraTable.getBooleanTopic("hasTarget").subscribe(false);
  private DoubleSubscriber pitch = cameraTable.getDoubleTopic("targetPitch").subscribe(defVal);
  private final double CAMERA_HEIGHT = 0.28;
  private final double REEF_TARGET = 0.22;
  private final double CAMERA_PITCH = 0;

  public AprilTagVisionIOPhotonvision() {}

  public void updateInputs(
      LoggableAprilTagVisionIOInputs loggableInputs,
      UnloggableAprilTagVisionIOInputs unloggableInputs) {
    loggableInputs.ntPose = poseSub.get();
    loggableInputs.ntYaw = yaw.get();
    loggableInputs.ntX = x.get();
    loggableInputs.ntY = y.get();
    loggableInputs.hasTarget = hasTarget.get();
    unloggableInputs.latestResult = camera.getLatestResult();
    unloggableInputs.latestEstimatedPose = updatePoseEstimator(unloggableInputs.latestResult);
    loggableInputs.latestCamToTagTranslation = getCamToTag(unloggableInputs.latestResult);
  }

  public Optional<EstimatedRobotPose> updatePoseEstimator(PhotonPipelineResult result) {
    return poseEstimator.update(result);
  }

  public Transform3d getCamToTag(PhotonPipelineResult result) {
    Transform3d camToTag = new Transform3d();
    if (result.hasTargets()) {
      camToTag = result.getBestTarget().getBestCameraToTarget();
    }
    return camToTag;
  }

  public double getXDistance() {
    double distance =
        (REEF_TARGET - CAMERA_HEIGHT)
            / Math.tan(Units.degreesToRadians(CAMERA_PITCH + pitch.get()));
    double x = distance * Units.degreesToRadians(yaw.get());
    return x;
  }

  @Override
  public double autoRotate() {
    System.out.println("R: " + rotatePid.calculate(yaw.get(), 0) * -0.2);
    return rotatePid.calculate(yaw.get(), 0) * -0.2;
  }

  @Override
  public double autoTranslateX() {
    System.out.println("X: " + -xPid.calculate(getXDistance(), 0));
    if (hasTarget.get()) {
      return -xPid.calculate(getXDistance(), 0);
    } else {
      return 0;
    }
  }

  @Override
  public double autoTranslateY() {
    System.out.println("Y: " + -yPid.calculate(y.get(), 10));
    if (hasTarget.get()) {
      return -yPid.calculate(y.get(), 10);
    } else {
      return 0;
    }
  }

  @Override
  public boolean hasTarget() {
    return hasTarget.get();
  }

  @Override
  public double getArea() {
    return y.get();
  }
}
