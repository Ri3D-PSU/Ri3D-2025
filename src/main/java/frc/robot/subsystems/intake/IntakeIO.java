package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double primaryAlgaeIntakeVoltage = 0.0;
    public double primaryAlgaeIntakeCurrent = 0.0;
    public double primaryAlgaeIntakeVelocity = 0.0;
    public double primaryAlgaeIntakePosition = 0.0;
    public double primaryAlgaeIntakeTemperature = 0.0;

    public double secondaryAlgaeIntakeCurrent = 0.0;
    public double secondaryAlgaeIntakeVoltage = 0.0;
    public double secondaryAlgaeIntakeTemperature = 0.0;
    public double secondaryAlgaeIntakeVelocity = 0.0;
    public double secondaryAlgaeIntakePosition = 0.0;

    public double coralIntakeVoltage = 0.0;
    public double coralIntakeCurrent = 0.0;
    public double coralIntakeVelocity = 0.0;
    public double coralIntakePosition = 0.0;
    public double coralIntakeTemperature = 0.0;

    public double coralWristCurrent = 0.0;
    public double coralWristVoltage = 0.0;
    public double coralWristTemperature = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPrimaryAlgaeIntakeVelocity(double velocity) {}

  public default void setSecondaryAlgaeIntakeVelocity(double velocity) {}

  public default void setCoralIntakeVelocity(double velocity) {}

  public default void setCoralWristVelocity(double velocity) {}

  public default void setCoralWristPosition(double position, double ffvoltage) {}

    public default void adjustAngle(double angleRadians) {}

    public default void wristAngle(double angleRadians) {}
}
