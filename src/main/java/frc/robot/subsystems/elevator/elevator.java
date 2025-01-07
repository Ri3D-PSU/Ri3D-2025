package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LoggedDashboardNumber elevatorStartPosition =
      new LoggedDashboardNumber("elevator Start Position", 0.0);

  private elevatorState state = elevatorState.IDLE;
  private double targetPower = 0.0;
  private double targetPosition = 0.0;

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  // Define elevator states
  public enum elevatorState {
    IDLE, // The elevator is stationary, no movement
    MOVING_TO_POSITION, // The elevator is moving to a specific target position
    AT_POSITION, // The elevator is at the desired target position
    MANUAL_CONTROL, // The elevator is being manually controlled
  }

  // Method to set power for the elevator
  public void setPower(double voltage) {
    state = elevatorState.MOVING_TO_POSITION;
    targetPower = voltage;
  }

  // Method to stop the elevator
  public void stop() {
    state = elevatorState.IDLE;
    targetPower = 0.0;
  }

  // Set the elevator to its maximum position
  public void setMax() {
    state = elevatorState.MOVING_TO_POSITION;
    targetPower = 0.5; // Or use a max power value
    targetPosition = 5.0; // Example of target position
  }

  // Set the elevator to a specific position
  public void setPosition(double position) {
    state = elevatorState.MOVING_TO_POSITION;
    targetPosition = position;
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getelevatorPosition() {
    return io.getPosition();
  }

  public double getelevatorVelocity() {
    return io.getVelocity();
  }

  public void resetelevatorPosition(double position) {
    io.resetPosition(position);
    elevatorStartPosition.set(position);
  }
}
