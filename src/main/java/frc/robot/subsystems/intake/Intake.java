package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  IntakeIO io;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setAlgaeVoltage(double voltage) {
    io.setAlgaeVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    io.setCoralIntakeVoltage(voltage);
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
    double wristffvoltage = feedforward.calculate(inputs.coralWristPosition, inputs.coralWristVelocity);
    io.setCoralWristPosition(targetPosition, wristffvoltage);
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