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

public class ElevatorSubsystem extends SubsystemBase {

  private final TurretSubsystem m_turretSubsystem;

  private TalonSRX m_elevatorMotor = new TalonSRX(Constants.ELEVATOR_TALON_PWM);

  public enum KnownElevatorPos {
    STOWED(10.0, 10.0),
    SETPOSE1(40.0, 20.0);

    public final double m_turretAngle;
    public final double m_elevatorPos;

    private KnownElevatorPos(double turretAngle, double elevatorPos) {
      m_turretAngle = turretAngle;
      m_elevatorPos = elevatorPos;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(TurretSubsystem turretSubsystem) {
    m_elevatorMotor.configFactoryDefault();
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

    m_turretSubsystem = turretSubsystem;
  }

  public void setTurrelavatorPos(final KnownElevatorPos pos) {
    double currentElevator = m_elevatorMotor.getSelectedSensorPosition();

    m_turretSubsystem.setTurretPos(pos.m_turretAngle);

    if (currentElevator < pos.m_elevatorPos) {
      m_elevatorMotor.configMotionAcceleration(Constants.ELEVATOR_VELO_UP, 0);
      m_elevatorMotor.configMotionCruiseVelocity(Constants.ELEVATOR_VELO_UP, 0);

      m_elevatorMotor.selectProfileSlot(0, 0);
    } else {
      m_elevatorMotor.configMotionAcceleration(Constants.ELEVATOR_VELO_DOWN, 0);
      m_elevatorMotor.configMotionCruiseVelocity(Constants.ELEVATOR_VELO_DOWN, 0);

      m_elevatorMotor.selectProfileSlot(1, 0);
    }

    m_elevatorMotor.set(TalonSRXControlMode.MotionMagic, pos.m_elevatorPos);
  }

  public void setElevatorPos(double targetPos) {
    m_elevatorMotor.set(TalonSRXControlMode.MotionMagic, targetPos);
  }

  public void setManualElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(TalonSRXControlMode.Current, elevatorSpeed);
  }

  public boolean isElevatorStalled() {
    return m_elevatorMotor.getSelectedSensorVelocity() < 500;
  }

  public void zeroElevator() {
    setManualElevatorSpeed(0.1);

    new WaitUntilCommand(this::isElevatorStalled);
    
    setManualElevatorSpeed(0);
    m_elevatorMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
  }
}
