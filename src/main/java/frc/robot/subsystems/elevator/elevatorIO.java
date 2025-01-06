package frc.robot.subsystems.elevator;

public interface elevatorIO {
  void setMotorOutput(double output);

  double getPosition(); // Get the elevator's current position

  void resetEncoder();

}

