// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ScoreHighBall;
import frc.robot.commands.autonomous.BallPickup;
import frc.robot.commands.autonomous.ClimbAuto;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.SubBallPickup;
import frc.robot.commands.autonomous.Turn90;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_primaryController = new CommandXboxController(0);

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
      m_primaryController.getLeftY() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      m_primaryController.getRightX() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      true),
      m_driveSubsystem));
  
    m_autoChooser.setDefaultOption("Do Nothing", () -> new DriveUntilCommand(m_driveSubsystem, 0,() -> true));

    m_autoChooser.addOption("2BallAvoid", () -> m_driveSubsystem.followTrajectoryCommand("2Ball Avoid"));

    m_autoChooser.addOption("2Ball Free", () -> m_driveSubsystem.followTrajectoryCommand("2Ball Free"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    NamedCommands.registerCommand("Turn 90", new Turn90(m_driveSubsystem, true));
    NamedCommands.registerCommand("Score High", new ScoreHighBall(m_elevatorSubsystem, m_intakeSubsystem));
    NamedCommands.registerCommand("Grab Ball", new BallPickup(m_elevatorSubsystem, m_intakeSubsystem));
    NamedCommands.registerCommand("Sub Ball", new SubBallPickup(m_elevatorSubsystem, m_intakeSubsystem));
    
    configureBindings();
  }

  private void configureBindings() {

    m_primaryController.povUp()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setManualElevatorSpeed(0.1), m_elevatorSubsystem));

    m_primaryController.povDown()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setManualElevatorSpeed(-0.1), m_elevatorSubsystem));

    m_primaryController.leftStick().and(m_primaryController.rightStick())
      .onTrue(new ClimbAuto(m_driveSubsystem, 1));
      
  }

  public Command getAutonomousCommand() {
    var autoCommandSupplier = m_autoChooser.getSelected();
    if (autoCommandSupplier != null) {
      return autoCommandSupplier.get()
        .beforeStarting(() -> m_elevatorSubsystem.zeroElevator(), m_elevatorSubsystem);
    }
    return null;
  }

  public void teleopInit() {
    m_elevatorSubsystem.zeroElevator();
  }
}
