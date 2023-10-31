// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.KnownElevatorPos;

public class BallPickup extends Command {

  private ElevatorSubsystem m_elevatorSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  public BallPickup(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(elevatorSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.BALLFLOOR);
    m_intakeSubsystem.setRollerSpeed(Constants.BALL_INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setRollerSpeed(0);
    m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.STOWED);
  }

  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.getRollerVelo() < 0.15;
  }
}
