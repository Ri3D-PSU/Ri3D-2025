package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class elevator extends SubsystemBase {

    private final elevatorIO io;
    private final elevatorIOInputsLogged inputs = new elevatorIOInputsLogged();

    private final LoggedDashboardNumber elevatorStartPosition = new LoggedDashboardNumber("elevator Start Position", 0.0);

    private elevatorState state = elevatorState.IDLE;
    private double targetPower = 0.0;
    private double targetPosition = 0.0;

    // Constructor
    public elevator(elevatorIO io) {
        this.io = io;
    }

    // Define elevator states
    public enum elevatorState {
        IDLE,               // The elevator is stationary, no movement
        MOVING_TO_POSITION, // The elevator is moving to a specific target position
        AT_POSITION,        // The elevator is at the desired target position
        MANUAL_CONTROL,     // The elevator is being manually controlled
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
        targetPower = 0.5;  // Or use a max power value
        targetPosition = 5.0;  // Example of target position
    }

    // Set the elevator to a specific position
    public void setPosition(double position) {
        state = elevatorState.MOVING_TO_POSITION;
        targetPosition = position;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    // Periodic method called in every cycle (e.g., 20ms)
    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                io.setPower(0.0); // No power applied to the elevator
                break;
            case MOVING_TO_POSITION:
                io.setPower(targetPower);
                if (Math.abs(io.getPosition() - targetPosition) < 0.1) {  // Tolerance for stopping
                    state = elevatorState.AT_POSITION;
                }
                break;
            case AT_POSITION:
                io.setPower(0.0); // Stop when at position
                break;
            case MANUAL_CONTROL:
                // Implement manual control logic if needed
                break;
        }

        // Log information for dashboard
    //     Logger.getInstance().recordOutput("elevator State", state.toString());
    //     Logger.getInstance().recordOutput("elevator Position", io.getelevatorPosition());
    //     Logger.getInstance().recordOutput("elevator Velocity", io.getelevatorVelocity());
    // }

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
