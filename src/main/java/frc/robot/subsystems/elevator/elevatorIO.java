package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double motorCurrent = 0;
    public double motorVoltage = 0;
    public double motorAngle = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
  // Sets the power to the elevator motor
  public default void set(double voltage) {}

  // Gets the current position of the elevator (in encoder units)
  public default double getPosition() {
    return 0;
  }

  public default void setPosition(double position) {}

  // Gets the current velocity of the elevator
  public default double getVelocity() {
    return 0;
  }

  // Resets the encoder position to a specific value
  public default void resetPosition() {}

  public default void stop() {}
}
