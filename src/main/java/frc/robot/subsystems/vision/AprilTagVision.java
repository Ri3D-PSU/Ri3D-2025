package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.UnloggableAprilTagVisionIOInputs;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVision extends SubsystemBase {

  private AprilTagVisionIO io;
  private LoggableAprilTagVisionIOInputsAutoLogged loggableIOInputs;
  private UnloggableAprilTagVisionIOInputs unloggableIOInputs;

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;
  }

  public double autoAlign() {
    return io.autoAlign();
  }

  public void periodic() {
    updateInputs();
  }

  public void updateInputs() {
    io.updateInputs(loggableIOInputs, unloggableIOInputs);
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return unloggableIOInputs.latestEstimatedPose;
  }

  public Transform3d getCamToTag() {
    return loggableIOInputs.latestCamToTagTranslation;
  }
}
