package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.imported.AHRS;
import java.util.OptionalDouble;
import java.util.Queue;

public class GyroIONavX implements GyroIO {

  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  double offset = 0;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> rate;

  public GyroIONavX() {
    rate =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = navx.isConnected() && !navx.isCalibrating();
                  if (valid) {
                    return OptionalDouble.of(Math.toRadians(navx.getRate()));
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = navx.isConnected() && !navx.isCalibrating();
                  if (valid) {
                    return OptionalDouble.of(navx.getRotation2d().getRadians());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
