// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private Talon m_elevatorMotor = new Talon(Constants.ELEVATOR_TALON_PWM);

  private Encoder m_elevatorEncoder = new Encoder(Constants.ELEVATOR_ENCODER_DIO_1, Constants.ELEVATOR_ENCODER_DIO_2);

  public enum KnownElevatorPos {
    STOWED(10.0, 10.0);

    public final double m_turretAngle;
    public final double m_elevatorPos;

    private KnownElevatorPos(double turretAngle, double elevatorPos) {
      m_turretAngle = turretAngle;
      m_elevatorPos = elevatorPos;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorEncoder.setDistancePerPulse(Constants.HD_HEX_COUNTS_PER_DEGREE);
  }

  public void setManualElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(elevatorSpeed);
  }

  public boolean isElevatorStalled() {
    return m_elevatorEncoder.getRate() < 500;
  }

  public void zeroElevator() {
    setManualElevatorSpeed(0.1);

    new WaitUntilCommand(this::isElevatorStalled);
    
    setManualElevatorSpeed(0);
    m_elevatorEncoder.reset();
  }

  @Override
  public void periodic() {
  }
}
