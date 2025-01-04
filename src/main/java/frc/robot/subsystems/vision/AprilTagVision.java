package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.UnloggableAprilTagVisionIOInputs;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVision extends SubsystemBase {

  private AprilTagVisionIO io;
  private LoggableAprilTagVisionIOInputsAutoLogged loggableInputs =
      new LoggableAprilTagVisionIOInputsAutoLogged();
  private UnloggableAprilTagVisionIOInputs unloggableInputs =
      new UnloggableAprilTagVisionIOInputs();

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;
  }

  public double autoRotate() {
    return io.autoRotate();
  }

  public double autoTranslateX() {
    return io.autoTranslateX();
  }

  public void periodic() {
    updateInputs();
  }

  public void updateInputs() {
    io.updateInputs(loggableInputs, unloggableInputs);
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return unloggableInputs.latestEstimatedPose;
  }

  public Transform3d getCamToTag() {
    return loggableInputs.latestCamToTagTranslation;
  }
}
