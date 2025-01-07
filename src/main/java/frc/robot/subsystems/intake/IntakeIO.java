package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double coralWristCurrent = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setAlgaeVoltage(double voltage) {}

  public default void setCoralIntakeVoltage(double voltage) {}

  public default void setCoralWristPosition(double position, double ffvoltage) {}

  public default void adjustAngle(double angleRadians) {}

  public default void wristAngle(double position) {}

  public default double getWristPosition() {
    return 0;
  }

  public default void setWristVoltage(double voltage) {}
}
