package frc.robot.subsystems.intake;

import static frc.robot.Constants.RPM_TO_RAD_PER_SEC;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  CANSparkMax algaeMotor1;
  CANSparkMax algaeMotor2;
  CANSparkMax coralIntake;
  CANSparkMax coralWrist;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new CANSparkMax(0, MotorType.kBrushless);
    algaeMotor2 = new CANSparkMax(0, MotorType.kBrushless);
    coralIntake = new CANSparkMax(0, MotorType.kBrushless);
    coralWrist = new CANSparkMax(0, MotorType.kBrushless);

    // ask about gear ratios for all motors
    coralWrist.getEncoder().setPositionConversionFactor(1.0 / 20.0);
    coralWrist.getEncoder().setVelocityConversionFactor((1.0 / 20.0) * RPM_TO_RAD_PER_SEC);
    coralWrist.enableVoltageCompensation(10.0);

    algaeMotor1.setSmartCurrentLimit(30);
    algaeMotor2.setSmartCurrentLimit(30);
    coralIntake.setSmartCurrentLimit(30);
    coralWrist.setSmartCurrentLimit(30);

    coralWrist.getPIDController().setP(0.4);
    coralWrist.getPIDController().setI(0.0001);
    coralWrist.getPIDController().setD(0.0);
    coralWrist.getPIDController().setFF(0);

    algaeMotor2.follow(algaeMotor1);

    algaeMotor1.setInverted(true);
    coralIntake.setInverted(true);
    coralWrist.setInverted(true);

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
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }
  
  public void setCoralWristPosition(double position, double ffvoltage) {
    coralWrist.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0,
                ffvoltage, ArbFFUnits.kVoltage);
  }

  @Override
    public void adjustAngle(double angleRadians) {
        coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
    }

    @Override
    public void wristAngle(double angleRadians) {
        coralWrist.getEncoder().setPosition(angleRadians);
    }
}
