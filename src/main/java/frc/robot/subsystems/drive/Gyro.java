package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Gyro {
  private GyroIO io;
  private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
  }

  public void reset() {
    io.resetGyro(inputs);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public Rotation2d[] getOdometryPositions() {
    return inputs.odometryYawPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryYawTimestamps;
  }
}
