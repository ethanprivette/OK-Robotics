// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Turn90 extends Command {

  private DriveSubsystem m_driveSubsystem;
  private boolean m_pos;
  private double desiredAngle;
  private PIDController m_pidContoller = new PIDController(0.0, 0.0, 0.0);

  /** Creates a new Turn90. */
  public Turn90(DriveSubsystem driveSubsystem, boolean positive) {
    m_driveSubsystem = driveSubsystem;
    m_pos = positive;

    m_pidContoller.enableContinuousInput(-180, 180);
    m_pidContoller.setTolerance(1.0);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidContoller.reset();
    if(m_pos) {
      desiredAngle = 90;
    } else if (!m_pos) {
      desiredAngle = -90;
    }
    desiredAngle = 0;
    System.out.println("ANGLE FAILED TO INITIALIZE");

    SmartDashboard.putNumber("Rotating to", desiredAngle);

    m_pidContoller.setSetpoint(desiredAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = m_driveSubsystem.getAngle();
    double rotRate = m_pidContoller.calculate(rawAngle);

    m_driveSubsystem.drive(0, rotRate, true);

    SmartDashboard.putNumber("Raw Angle", rawAngle);
    SmartDashboard.putNumber("Rotation Rate", rotRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidContoller.atSetpoint();
  }
}
