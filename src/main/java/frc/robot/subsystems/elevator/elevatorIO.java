package frc.robot.subsystems.elevator;

public interface elevatorIO {
    // Sets the power to the elevator motor
    void setPower(double power);

    // Gets the current position of the elevator (in encoder units)
    double getPosition();

    // Gets the current velocity of the elevator
    double getVelocity();

    // Resets the encoder position to a specific value
    void resetPosition(double position);
}
