// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final double LIFT_VOLTAGE = 10;

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  public void turnMotorUntilAngle(double angle) {
    turnMotorUntilAngle(LIFT_VOLTAGE, angle);
  }

  public void turnMotorUntilAngle(double lift_voltage, double angle) {
    if(inputs.motorAngle < angle) {
      io.setMotorVoltage(lift_voltage);
    } else {
      io.setMotorVoltage(0);
    }
  }

  public void turnMotorUntilCurrent(double current) {
    turnMotorUntilCurrent(LIFT_VOLTAGE, current);
  }
  
  public void turnMotorUntilCurrent(double lift_voltage, double current) {
    if(inputs.motorCurrent < current) {
      io.setMotorVoltage(lift_voltage);
    } else {
      io.setMotorVoltage(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }
}
