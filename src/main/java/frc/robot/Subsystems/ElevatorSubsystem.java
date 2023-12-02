// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private PWMVictorSPX m_elevatorMotor = new PWMVictorSPX(Constants.ELEVATOR_TALON_PWM);

  private final PIDController m_positionController = new PIDController(0, 0, 0);

  private Encoder m_elevatorEncoder = new Encoder(Constants.ELEVATOR_ENCODER_DIO_1, Constants.ELEVATOR_ENCODER_DIO_2);

  private double m_targetSetpoint;

  public enum KnownElevatorPos {
    STOWED(150.0),
    BALLFLOOR(220.0),
    BALLSUB(150.0),
    SCORELOWBALL(170.0),
    SCOREHIGHBALL(25.0),
    FLAGSUB(25.0),
    FLAGLOW(100.0),
    FLAGHIGH(5.0),
    TEST1(70.0),
    TEST2(170);

    public final double m_elevatorPos;

    private KnownElevatorPos(double elevatorPos) {
      m_elevatorPos = elevatorPos;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_positionController.disableContinuousInput();
    m_positionController.setTolerance(1);
  }

  public void setElavatorPos(final KnownElevatorPos pos) {
    m_targetSetpoint = pos.m_elevatorPos;
  }

  public void proceedToElevatorPos() {
    double currentDegrees = m_elevatorEncoder.get();

    if (m_targetSetpoint < currentDegrees) {
      m_positionController.setP(0.15);
    } else {
      m_positionController.setP(0.1);
    }

    double output = m_positionController.calculate(currentDegrees, m_targetSetpoint);
    m_elevatorMotor.set(-output);
  }

  public void nudgeElevator(double ticks) {
    m_targetSetpoint += ticks;
  }

  public void setManualElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(elevatorSpeed);
  }

  public boolean isElevatorStalled() {
    return m_elevatorEncoder.getStopped();
  }

  public void resetPID() {
    m_positionController.setSetpoint(0.0);
  }

  public void resetEncoder() {
    m_elevatorEncoder.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Pos", m_elevatorEncoder.get());
    SmartDashboard.putNumber("Elevator Encoder Rate", m_elevatorEncoder.getRate());
    SmartDashboard.putBoolean("Elevator Encoder Stopped", m_elevatorEncoder.getStopped());
    SmartDashboard.putNumber("Elevator Target Setpoint", m_targetSetpoint);
  }
}
