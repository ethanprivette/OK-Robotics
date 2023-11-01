// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX m_intakeMotor = new TalonSRX(Constants.INTAKE_TALON_PWM);

  public IntakeSubsystem() {
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
  }

  public void setRollerSpeed(double speed) {
    m_intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public double getRollerVelo() {
    return m_intakeMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
