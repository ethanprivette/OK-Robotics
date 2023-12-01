// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private AHRS m_gyro;

  private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_DIO_1, Constants.LEFT_ENCODER_DIO_2);
  private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_DIO_1, Constants.RIGHT_ENCODER_DIO_2);

  private PWMTalonSRX m_left = new PWMTalonSRX(Constants.LEFT_TALON_PWM);
  private PWMTalonSRX m_right = new PWMTalonSRX(Constants.RIGHT_TALON_PWM);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private DifferentialDriveOdometry m_odometry;

  private Field2d m_field = new Field2d();

  private double m_xMetersPerSec = 0;
  private double m_zRadPerSec = 0;

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

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    m_xMetersPerSec = speeds.vxMetersPerSecond;
    m_zRadPerSec = speeds.omegaRadiansPerSecond;
    
    m_drive.curvatureDrive(m_xMetersPerSec, m_zRadPerSec, true);

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

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(m_xMetersPerSec, 0, m_zRadPerSec);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Something", m_gyro.getRoll());

    m_field.setRobotPose(getPose());
  }

  public Command followTrajectoryCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathWithEvents(
      new FollowPathRamsete(
        path,
        this::getPose, 
        this::getChassisSpeeds,
        this::driveChassisSpeeds,
        new ReplanningConfig(),
        this
      ),
      path,
      this::getPose
    );
  }

}