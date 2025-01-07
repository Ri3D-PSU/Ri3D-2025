package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final CANSparkMax leadMotor;
  private final CANSparkMax followerMotor;
  private final RelativeEncoder encoder;

  // Constructor
  public ElevatorIOSparkMax(int leadMotorID, int followerMotorID) {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new CANSparkMax(leadMotorID, MotorType.kBrushless);
    followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);

    // Invert follower
    followerMotor.setInverted(true);
    followerMotor.follow(leadMotor);

    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();
  }

  @Override
  public void setPower(double power) {
    // Set the power to the main motor
    leadMotor.set(power);
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
  public void resetPosition(double position) {
    // Reset the encoder to the specified position
    encoder.setPosition(position);
  }
}
