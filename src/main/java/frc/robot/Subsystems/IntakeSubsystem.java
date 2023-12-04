// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private PWMTalonSRX m_intakeMotor = new PWMTalonSRX(Constants.INTAKE_TALON_PWM);

  public IntakeSubsystem() {
    m_intakeMotor.setInverted(true);
  }

  public void setRollerSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public double getRollerVelo() {
    return m_intakeMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
