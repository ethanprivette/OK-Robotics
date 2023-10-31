// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.KnownElevatorPos;

public class ScoreHighBall extends Command {

  private ElevatorSubsystem m_elevatorSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new ScoreHighBall. */
  public ScoreHighBall(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(elevatorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.SCOREHIGHBALL);
    new WaitCommand(1);
    m_intakeSubsystem.setRollerSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.getRollerVelo() > 0.75;
  }
}
