package frc.robot.subsystems.elevator;

public interface elevatorIO {
    // Sets the power to the elevator motor
    void setelevatorPower(double power);

    // Gets the current position of the elevator (in encoder units)
    double getelevatorPosition();

    // Gets the current velocity of the elevator
    double getelevatorVelocity();

    // Resets the encoder position to a specific value
    void resetelevatorPosition(double position);
}
