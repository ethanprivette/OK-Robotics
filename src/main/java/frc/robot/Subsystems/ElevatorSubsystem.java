// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private VictorSPX m_elevatorMotor = new VictorSPX(Constants.ELEVATOR_TALON_PWM);

  private Encoder m_elevatorEncoder = new Encoder(Constants.ELEVATOR_ENCODER_DIO_1, Constants.ELEVATOR_ENCODER_DIO_2);

  public enum KnownElevatorPos {
    STOWED( 10.0),
    BALLFLOOR(1.0),
    BALLSUB(15.0),
    SETPOSE1( 20.0),
    SCOREHIGHBALL(40.0);

    public final double m_elevatorPos;

    private KnownElevatorPos(double elevatorPos) {
      m_elevatorPos = elevatorPos;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor.configFactoryDefault();
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_elevatorMotor.configMotionAcceleration(1.0, 0);
  }

  public void setElavatorPos(final KnownElevatorPos pos) {
    m_elevatorMotor.set(VictorSPXControlMode.MotionMagic, pos.m_elevatorPos);
  }

  public void setManualElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(VictorSPXControlMode.PercentOutput, elevatorSpeed);
  }

  public boolean isElevatorStalled() {
    return m_elevatorEncoder.getRate() < 8;
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
