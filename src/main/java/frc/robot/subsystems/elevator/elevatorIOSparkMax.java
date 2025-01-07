package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final CANSparkMax leadMotor;
  private final CANSparkMax followerMotor;
  private final RelativeEncoder encoder;

  // Constructor
  public ElevatorIOSparkMax() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new CANSparkMax(14, MotorType.kBrushless);
    followerMotor = new CANSparkMax(24, MotorType.kBrushless);

    // Invert follower
    followerMotor.setInverted(true);
    followerMotor.follow(leadMotor, true);

    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();

    leadMotor.getPIDController().setP(0.027);
    leadMotor.getPIDController().setI(0);
    leadMotor.getPIDController().setD(0);
    leadMotor.getPIDController().setFF(0.0085);

    leadMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setIdleMode(IdleMode.kBrake);

    leadMotor.burnFlash();
    followerMotor.burnFlash();
  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}
