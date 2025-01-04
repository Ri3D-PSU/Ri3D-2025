package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagVisionIO {
  @AutoLog
  public static class LoggableAprilTagVisionIOInputs {
    public double[] ntPose = {0, 0, 0, 0, 0, 0};
    public double ntYaw = 0;
    public Transform3d latestCamToTagTranslation = new Transform3d();
  }

  // Autolog doesn't work on these types
  public static class UnloggableAprilTagVisionIOInputs {
    public PhotonPipelineResult latestResult = null;
    public Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
  }

  public default void updateInputs(
      LoggableAprilTagVisionIOInputs loggableInputs,
      UnloggableAprilTagVisionIOInputs unloggableInputs) {}
  ;

  public default Optional<EstimatedRobotPose> updatePoseEstimator(
      List<PhotonPipelineResult> results) {
    return Optional.empty();
  }
  ;

  public default double autoAlign() {
    return 0;
  }

  public default Transform3d getCamToTag(List<PhotonPipelineResult> results) {
    return new Transform3d();
  }
  ;
}
