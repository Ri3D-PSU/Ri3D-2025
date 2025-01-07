package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface elevatorIO {
    @AutoLog
    public static class elevatorIOInputs {
      public double motorCurrent = 0;
      public double motorVoltage = 0;
      public double motorAngle = 0;
    }
    public default void updateInputs(elevatorIOInputs inputs) {}
    // Sets the power to the elevator motor
    void setPower(double power);

    // Gets the current position of the elevator (in encoder units)
    double getPosition();

    // Gets the current velocity of the elevator
    double getVelocity();

    // Resets the encoder position to a specific value
    void resetPosition(double position);
}
