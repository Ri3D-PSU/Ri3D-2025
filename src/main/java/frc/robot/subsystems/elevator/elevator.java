package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;

  // PID coefficients (for ltr)

  // Elevator height limits (in encoder units)
  private static final double kMinHeight = 0.0; // Bottom position
  private static final double kMaxHeight = 100.0; // Top position

  public elevator(int motorID) {
    elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();

    // Reset encoder to ensure starting position is zero
    elevatorEncoder.setPosition(0.0);
  }

  public void goToPosition(double position) {
    // Clamp the position to be within the height limits
    double clampedPosition = Math.max(kMinHeight, Math.min(position, kMaxHeight));
  }

  public void resetPosition(double position) {
    elevatorEncoder.setPosition(position);
    System.out.println("Elevator position reset to: " + position);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void stop() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // Periodically log the elevator's position for debugging
    System.out.println("Elevator Position: " + getPosition());
  }
}
