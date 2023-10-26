// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private AHRS m_gyro;

  private Talon m_left = new Talon(Constants.LEFT_TALON_PWM);
  private Talon m_right = new Talon(Constants.RIGHT_TALON_PWM);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro = new AHRS();
    m_right.setInverted(true);
  }

  public void drive(double xRequest, double rotRequest, boolean turnInPlace) {
    m_drive.curvatureDrive(xRequest, rotRequest, turnInPlace);
  }

  public boolean onFlat() {
    return Math.abs(m_gyro.getPitch()) < 4;
  }

  public double getAngle() {
    return m_gyro.getYaw() + 180;
  }

  @Override
  public void periodic() {
 
  }
}