// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private AHRS m_gyro;

  private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_DIO_1, Constants.LEFT_ENCODER_DIO_2);
  private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_DIO_1, Constants.RIGHT_ENCODER_DIO_2);

  private Talon m_left = new Talon(Constants.LEFT_TALON_PWM);
  private Talon m_right = new Talon(Constants.RIGHT_TALON_PWM);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private DifferentialDriveOdometry m_odometry;

  private Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro = new AHRS();

    m_right.setInverted(true);

    m_odometry = new DifferentialDriveOdometry(getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      new Pose2d());

    SmartDashboard.putData("Field", m_field);
  }

  public void drive(double xRequest, double rotRequest, boolean turnInPlace) {
    m_drive.curvatureDrive(xRequest, rotRequest, turnInPlace);
    
    updateOdometry();
  }

  public void updateOdometry() {
    m_odometry.update(getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public boolean onFlat() {
    return Math.abs(m_gyro.getPitch()) < 4;
  }

  public double getAngle() {
    return m_gyro.getYaw() + 180;
  }

  @Override
  public void periodic() {
    

    m_field.setRobotPose(getPose());
  }
}