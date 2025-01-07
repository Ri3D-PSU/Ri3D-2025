// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
  private final int MOTOR_GEAR_RATIO = 405;

  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private RelativeEncoder motorRelativeEncoder;

  public ClimberIOSparkMax() {
    motor1 = new CANSparkMax(13, MotorType.kBrushless);
    motor2 = new CANSparkMax(23, MotorType.kBrushless);

    motor1.restoreFactoryDefaults();

    motor1.setCANTimeout(250);
    motor1.setSmartCurrentLimit(40);
    motor1.enableVoltageCompensation(12);
    motor1.setIdleMode(IdleMode.kBrake);

    motorRelativeEncoder = motor1.getEncoder();
    motorRelativeEncoder.setPositionConversionFactor(1. / MOTOR_GEAR_RATIO);

    motor2.follow(motor1, true);

    motor1.burnFlash();
    motor2.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorAngle = motorRelativeEncoder.getPosition();
    inputs.motorVoltage = motor1.getBusVoltage();
    inputs.motorCurrent = motor1.getOutputCurrent();
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor1.setVoltage(volts);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }
}
