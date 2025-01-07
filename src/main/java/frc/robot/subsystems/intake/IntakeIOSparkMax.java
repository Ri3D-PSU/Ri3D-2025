package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  CANSparkMax algaeMotor1;
  CANSparkMax algaeMotor2;
  CANSparkMax coralIntake;
  CANSparkMax coralWrist;
  RelativeEncoder wristEncoder;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new CANSparkMax(17, MotorType.kBrushless);
    algaeMotor2 = new CANSparkMax(27, MotorType.kBrushless);
    coralIntake = new CANSparkMax(15, MotorType.kBrushless);
    coralWrist = new CANSparkMax(16, MotorType.kBrushless); // dont have yet

    // ask about gear ratios for all motors
    wristEncoder = coralWrist.getEncoder();

    algaeMotor1.restoreFactoryDefaults();
    algaeMotor2.restoreFactoryDefaults();

    algaeMotor1.setSmartCurrentLimit(15);
    algaeMotor2.setSmartCurrentLimit(15);
    coralIntake.setSmartCurrentLimit(15);
    coralWrist.setSmartCurrentLimit(40);

    coralWrist.getPIDController().setP(0.55);
    coralWrist.getPIDController().setI(0);
    coralWrist.getPIDController().setD(0.0);
    coralWrist.getPIDController().setFF(0.00375);

    algaeMotor1.setIdleMode(IdleMode.kBrake);
    algaeMotor2.setIdleMode(IdleMode.kBrake);
    coralIntake.setIdleMode(IdleMode.kBrake);
    coralWrist.setIdleMode(IdleMode.kBrake);

    algaeMotor1.burnFlash();
    algaeMotor2.burnFlash();
    coralIntake.burnFlash();
    coralWrist.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(-voltage);
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  @Override
  public void adjustAngle(double angleRadians) {
    coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
  }

  @Override
  public void wristAngle(double position) {
    coralWrist.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public void setWristVoltage(double voltage) {
    System.out.println("Wrist position: " + getWristPosition());
    coralWrist.set(voltage);
  }
}
