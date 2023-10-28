// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private TalonSRX m_turretMotor = new TalonSRX(Constants.TURRET_TALON_PWM);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition); 
  }

  public void setTurretPos(double angle) {
    double currentTurrentAngle = m_turretMotor.getSelectedSensorPosition();

    m_turretMotor.configMotionAcceleration(Constants.TURRET_ACCELERATION, 0);
    m_turretMotor.configMotionCruiseVelocity(Constants.TURRET_VELO, 0);

    m_turretMotor.set(TalonSRXControlMode.MotionMagic, angle);
  }

  public void setManualTurretSpeed(double turretSpeed) {
    m_turretMotor.set(TalonSRXControlMode.Current, turretSpeed);
  }

  public boolean isTurretStalled() {
    return m_turretMotor.getSelectedSensorVelocity() < 500;
  }
  
  public void zeroTurret() {
    setManualTurretSpeed(0.1);

    new WaitUntilCommand(this::isTurretStalled);

    setManualTurretSpeed(0);
    m_turretMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
