package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  IntakeIO io;
  IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setPrimaryAlgaeIntakeVelocity(double velocity) {
    io.setPrimaryAlgaeIntakeVelocity(velocity);
    Logger.getInstance().recordOutput("PrimaryAlgaeIntakeTargetVelocity", velocity);
  }

  public void setSecondaryAlgaeIntakeVelocity(double velocity) {
    io.setSecondaryAlgaeIntakeVelocity(velocity);
    Logger.getInstance().recordOutput("SecondaryAlgaeIntakeTargetVelocity", velocity);
  }

  public void setCoralIntakeVelocity(double velocity) {
    io.setCoralIntakeVelocity(velocity);
    Logger.getInstance().recordOutput("CoralIntakeTargetVelocity", velocity);
  }

  public void setCoralWristVelocity(double velocity) {
    io.setCoralWristVelocity(velocity);
    Logger.getInstance().recordOutput("CoralWristTargetVelocity", velocity);
  }

  private double targetPosition = 0.0;
    private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);

  public void setWristPositionDegrees(double position) {
    double targetPosition = Math.toRadians(position);
}

public double getTargetWristPosition() {
    return targetPosition;
}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    double wristffvoltage = feedforward.calculate(inputs.coralWristPosition, inputs.coralWristVelocity);
        Logger.getInstance().recordOutput("ArmFFVoltage", wristffvoltage);

        io.setCoralWristPosition(targetPosition, wristffvoltage);
  }

  public double getAlgaeIntakeVelocity() {
    return inputs.primaryAlgaeIntakeVelocity;
  }

  public double getCoralIntakeVelocity() {
    return inputs.coralIntakeVelocity;
  }

  public double getCoralWristVelocity() {
    return inputs.coralWristVoltage;
  }

  public double getAlgaeIntakeCurrent() {
    return inputs.primaryAlgaeIntakeCurrent;
  }

  public double getCoralIntakeCurrent() {
    return inputs.coralIntakeCurrent;
  }

  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristCurrent;
  }

  public double getWristPosition() {
        return inputs.coralWristPosition;
    }

    public void adjustAngle(double degrees) {
        io.adjustAngle(Math.toRadians(degrees));
    }

    public void resetAngle(double radians) {

    }
}