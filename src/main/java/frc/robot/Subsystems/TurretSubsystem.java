// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private Talon m_turretMotor = new Talon(Constants.TURRET_TALON_PWM);

  private Encoder m_turretEncoder = new Encoder(Constants.TURRET_ENCODER_DIO_1, Constants.TURRET_ENCODER_DIO_2);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {}

  public void setManualTurretSpeed(double turretSpeed) {
    m_turretMotor.set(turretSpeed);
  }

  public boolean isTurretStalled() {
    return m_turretEncoder.getRate() < 500;
  }
  
  public void zeroTurret() {
    setManualTurretSpeed(0.1);

    new WaitUntilCommand(this::isTurretStalled);

    setManualTurretSpeed(0);
    m_turretEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
