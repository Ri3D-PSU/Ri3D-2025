package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class elevatorIOSparkMax implements elevatorIO {
    private final CANSparkMax batmanMotor;
    private final CANSparkMax robinMotor;
    private final RelativeEncoder batmanEncoder;

    // Constructor
    public elevatorIOSparkMax(int batmanMotorID, int robinMotorID) {
        // Initialize the CANSparkMax motors for Batman (main) and Robin (follower)
        batmanMotor = new CANSparkMax(batmanMotorID, MotorType.kBrushless);
        robinMotor = new CANSparkMax(robinMotorID, MotorType.kBrushless);

        // Invert Robin (follower motor)
        robinMotor.setInverted(true);

        // Make Robin follow Batman motor
        robinMotor.follow(batmanMotor);

        // Initialize the encoder for Batman (main motor)
        batmanEncoder = batmanMotor.getEncoder();
    }

    @Override
    public void setelevatorPower(double power) {
        // Set the power to the main motor (Batman)
        batmanMotor.set(power);
    }

    @Override
    public double getelevatorPosition() {
        // Get the position from the encoder on the Batman motor
        return batmanEncoder.getPosition();
    }

    @Override
    public double getelevatorVelocity() {
        // Get the velocity from the encoder on the Batman motor
        return batmanEncoder.getVelocity();
    }

    @Override
    public void resetelevatorPosition(double position) {
        // Reset the encoder to the specified position
        batmanEncoder.setPosition(position);
    }
}
