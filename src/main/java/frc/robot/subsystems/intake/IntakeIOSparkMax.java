package frc.robot.subsystems.intake;

import static frc.robot.Constants.RPM_TO_RAD_PER_SEC;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {
  CANSparkMax primaryAlgaeIntake;
  CANSparkMax secondaryAlgaeIntake;
  CANSparkMax coralIntake;
  CANSparkMax coralWrist;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    primaryAlgaeIntake = new CANSparkMax(1, MotorType.kBrushless);
    secondaryAlgaeIntake = new CANSparkMax(2, MotorType.kBrushless);
    coralIntake = new CANSparkMax(3, MotorType.kBrushless);
    coralWrist = new CANSparkMax(4, MotorType.kBrushless);

    // ask about gear ratios for all motors
    primaryAlgaeIntake.getEncoder().setPositionConversionFactor(1.0 / 5.0);
    secondaryAlgaeIntake.getEncoder().setPositionConversionFactor(1.0 / 20.0);
    coralIntake.getEncoder().setPositionConversionFactor(1.0 / 5.0);
    coralWrist.getEncoder().setPositionConversionFactor(1.0 / 20.0);

    primaryAlgaeIntake.getEncoder().setVelocityConversionFactor((1.0 / 5.0) * RPM_TO_RAD_PER_SEC);
    secondaryAlgaeIntake
        .getEncoder()
        .setVelocityConversionFactor((1.0 / 20.0) * RPM_TO_RAD_PER_SEC);
    coralIntake.getEncoder().setVelocityConversionFactor((1.0 / 5.0) * RPM_TO_RAD_PER_SEC);
    coralWrist.getEncoder().setVelocityConversionFactor((1.0 / 20.0) * RPM_TO_RAD_PER_SEC);

    primaryAlgaeIntake.enableVoltageCompensation(10.0);
    secondaryAlgaeIntake.enableVoltageCompensation(10.0);
    coralIntake.enableVoltageCompensation(10.0);
    coralWrist.enableVoltageCompensation(10.0);

    primaryAlgaeIntake.setSmartCurrentLimit(30);
    secondaryAlgaeIntake.setSmartCurrentLimit(30);
    coralIntake.setSmartCurrentLimit(30);
    coralWrist.setSmartCurrentLimit(30);

    // Controls
    primaryAlgaeIntake.getPIDController().setP(0.00);
    primaryAlgaeIntake.getPIDController().setI(0.00000);
    primaryAlgaeIntake.getPIDController().setD(0.0);
    primaryAlgaeIntake.getPIDController().setFF(1.0 / 1000.0);

    secondaryAlgaeIntake.getPIDController().setP(0.00);
    secondaryAlgaeIntake.getPIDController().setI(0.00000);
    secondaryAlgaeIntake.getPIDController().setD(0.0);
    secondaryAlgaeIntake.getPIDController().setFF(1.0 / 1000.0);

    coralIntake.getPIDController().setP(0.00);
    coralIntake.getPIDController().setI(0.00000);
    coralIntake.getPIDController().setD(0.0);
    coralIntake.getPIDController().setFF(1.0 / 1000.0);

    coralWrist.getPIDController().setP(0.4);
    coralWrist.getPIDController().setI(0.0001);
    coralWrist.getPIDController().setD(0.0);
    coralWrist.getPIDController().setFF(0);

    primaryAlgaeIntake.setInverted(true);
    secondaryAlgaeIntake.setInverted(false);
    coralIntake.setInverted(true);
    coralWrist.setInverted(true);

    primaryAlgaeIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    secondaryAlgaeIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    coralIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    coralWrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.primaryAlgaeIntakeCurrent = primaryAlgaeIntake.getOutputCurrent();
    inputs.primaryAlgaeIntakeVoltage = primaryAlgaeIntake.getBusVoltage();
    inputs.primaryAlgaeIntakeVelocity = primaryAlgaeIntake.getEncoder().getVelocity();
    inputs.primaryAlgaeIntakePosition = primaryAlgaeIntake.getEncoder().getPosition();
    inputs.primaryAlgaeIntakeTemperature = primaryAlgaeIntake.getMotorTemperature();

    inputs.secondaryAlgaeIntakeCurrent = secondaryAlgaeIntake.getOutputCurrent();
    inputs.secondaryAlgaeIntakeVoltage = secondaryAlgaeIntake.getBusVoltage();
    inputs.secondaryAlgaeIntakeVelocity = secondaryAlgaeIntake.getEncoder().getVelocity();
    inputs.secondaryAlgaeIntakePosition = secondaryAlgaeIntake.getEncoder().getPosition();
    inputs.secondaryAlgaeIntakeTemperature = secondaryAlgaeIntake.getMotorTemperature();

    inputs.coralIntakeCurrent = coralIntake.getOutputCurrent();
    inputs.coralIntakeVoltage = coralIntake.getBusVoltage();
    inputs.coralIntakeVelocity = coralIntake.getEncoder().getVelocity();
    inputs.coralIntakePosition = coralIntake.getEncoder().getPosition();
    inputs.coralIntakeTemperature = coralIntake.getMotorTemperature();

    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVoltage = coralWrist.getBusVoltage();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
    inputs.coralWristTemperature = coralWrist.getMotorTemperature();
  }

  @Override
  public void setPrimaryAlgaeIntakeVelocity(double velocity) {
    primaryAlgaeIntake.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void setSecondaryAlgaeIntakeVelocity(double velocity) {
    secondaryAlgaeIntake.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void setCoralIntakeVelocity(double velocity) {
    coralIntake.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void setCoralWristVelocity(double velocity) {
    coralWrist.getPIDController().setReference(velocity, ControlType.kVelocity);
  }
}
