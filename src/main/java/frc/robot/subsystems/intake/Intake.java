package frc.robot.subsystems.intake;

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }

  public double getPrimaryAlgaeIntakeVelocity() {
    return inputs.primaryAlgaeIntakeVelocity;
  }

  public double getSecondaryAlgaeIntakeVelocity() {
    return inputs.secondaryAlgaeIntakeVelocity;
  }

  public double getCoralIntakeVelocity() {
    return inputs.coralIntakeVelocity;
  }

  public double getCoralWristVelocity() {
    return inputs.coralWristVoltage;
  }

  public double getPrimaryAlgaeIntakeCurrent() {
    return inputs.primaryAlgaeIntakeCurrent;
  }

  public double getSecondaryAlgaeIntakeCurrent() {
    return inputs.secondaryAlgaeIntakeCurrent;
  }

  public double getCoralIntakeCurrent() {
    return inputs.coralIntakeCurrent;
  }

  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristCurrent;
  }
}
