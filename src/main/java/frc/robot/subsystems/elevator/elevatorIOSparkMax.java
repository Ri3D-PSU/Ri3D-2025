package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class elevatorIOSparkMax implements elevatorIO {
  private final CANSparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;

  public elevatorIOSparkMax(int motorID) {
    elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();

    // Reset the encoder to zero at initialization
    resetEncoder();
  }

  @Override
  public void setMotorOutput(double output) {
    elevatorMotor.set(output);
  }

  @Override
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void resetEncoder() {
    elevatorEncoder.setPosition(0.0);
  }
}
