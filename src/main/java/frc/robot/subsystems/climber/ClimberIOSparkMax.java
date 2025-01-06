// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
    private final int MOTOR_GEAR_RATIO = 405;
    private CANSparkMax motor;
    private RelativeEncoder motorRelativeEncoder;

    public ClimberIOSparkMax(int motorID) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        
        motor.setCANTimeout(250);
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kBrake);

        motorRelativeEncoder = motor.getEncoder();
        motorRelativeEncoder.setPositionConversionFactor(1. / MOTOR_GEAR_RATIO);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorAngle = motorRelativeEncoder.getPosition();
        inputs.motorCurrent = motor.getOutputCurrent();
    }

    @Override
    public void setMotorVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
