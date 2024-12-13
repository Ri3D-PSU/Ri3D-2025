package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;

public interface AprilTagVisionIO {
    public enum TargetType {APRILTAG, NOTE}

    @AutoLog
    public static class VisionIOInputs{
        public TargetType targetType;
        public double[] ntPose = {0, 0, 0, 0, 0, 0};
        public List<PhotonPipelineResult> unreadResults = null;
        public Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
        public Transform3d latestCamToTargetTranslation = new Transform3d();
    }
}
