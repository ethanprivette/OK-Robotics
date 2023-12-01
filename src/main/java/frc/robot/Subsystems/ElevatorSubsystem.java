// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private PWMVictorSPX m_elevatorMotor = new PWMVictorSPX(Constants.ELEVATOR_TALON_PWM);

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
  }

  public void setElavatorPos(final KnownElevatorPos pos) {
  }

  public void setManualElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(elevatorSpeed);
  }

  public boolean isElevatorStalled() {
    return m_elevatorEncoder.getStopped();
  }

  public void zeroElevator() {
    setManualElevatorSpeed(0.4);

    new WaitCommand(2.5);
    
    setManualElevatorSpeed(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Pos", m_elevatorEncoder.get());
    SmartDashboard.putNumber("Elevator Encoder Rate", m_elevatorEncoder.getRate());
    SmartDashboard.putBoolean("Elevator Encoder Stopped", m_elevatorEncoder.getStopped());
  }
}
