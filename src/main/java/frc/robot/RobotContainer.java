// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ScoreHighBall;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.autonomous.BallPickup;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.SubBallPickup;
import frc.robot.commands.autonomous.Turn90;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.KnownElevatorPos;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_primaryController = new CommandXboxController(0);

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

  private boolean ballMode = true;

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
      modifyAxis(m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(m_primaryController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      true),
      m_driveSubsystem));

    m_elevatorSubsystem.setDefaultCommand(
      new RunCommand(() -> m_elevatorSubsystem.proceedToElevatorPos(), m_elevatorSubsystem));
  
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

    m_primaryController.a().and(() -> ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.SCORELOWBALL), m_elevatorSubsystem));

    m_primaryController.a().and(() -> !ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.FLAGLOW), m_elevatorSubsystem));

    m_primaryController.b().and(() -> ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.BALLSUB), m_elevatorSubsystem));

    m_primaryController.a().and(() -> !ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.FLAGSUB), m_elevatorSubsystem));

    m_primaryController.x()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.STOWED), m_elevatorSubsystem));

    m_primaryController.y().and(() -> ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.SCOREHIGHBALL), m_elevatorSubsystem));

    m_primaryController.y().and(() -> !ballMode)
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.FLAGHIGH), m_elevatorSubsystem));

    m_primaryController.rightBumper().and(() -> ballMode)
      .whileTrue(new InstantCommand(() -> m_intakeSubsystem.setRollerSpeed(0.5), m_intakeSubsystem));

    m_primaryController.leftBumper().and(() -> ballMode)
      .whileTrue(new InstantCommand(() -> m_intakeSubsystem.setRollerSpeed(-0.5), m_intakeSubsystem));

    m_primaryController.rightBumper().and(() -> !ballMode)
      .whileTrue(new InstantCommand(() -> m_intakeSubsystem.setRollerSpeed(-0.5), m_intakeSubsystem));

    m_primaryController.leftBumper().and(() -> !ballMode)
      .whileTrue(new InstantCommand(() -> m_intakeSubsystem.setRollerSpeed(0.5), m_intakeSubsystem));

    m_primaryController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.BALLFLOOR), m_elevatorSubsystem)
      .alongWith(new InstantCommand(() -> changeBallMode(true))));

    m_primaryController.start()
      .onTrue(new InstantCommand(() -> changeBallMode(false)));   

    m_primaryController.back()
      .onTrue(new InstantCommand(() -> changeBallMode(true)));

    m_primaryController.rightStick()
      .whileTrue(new ZeroElevatorCommand(m_elevatorSubsystem));

    m_primaryController.povUp()
      .onTrue(new InstantCommand(() ->
        m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.TEST1), 
        m_elevatorSubsystem));

    m_primaryController.povDown()
      .onTrue(new InstantCommand(() ->
        m_elevatorSubsystem.setElavatorPos(KnownElevatorPos.TEST2), 
        m_elevatorSubsystem));

    m_primaryController.povRight()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.nudgeElevator(5), m_elevatorSubsystem));

    m_primaryController.povLeft()
      .onTrue(new InstantCommand(() -> m_elevatorSubsystem.nudgeElevator(-5), m_elevatorSubsystem));

    m_intakeSubsystem.setDefaultCommand(
      new RunCommand(() -> m_intakeSubsystem.setRollerSpeed(0.0), m_intakeSubsystem));
  }

  private void changeBallMode(boolean bool) {
    ballMode = bool;
  }

  public Command getAutonomousCommand() {
    var autoCommandSupplier = m_autoChooser.getSelected();
    if (autoCommandSupplier != null) {
      return autoCommandSupplier.get()
        .beforeStarting(new ZeroElevatorCommand(m_elevatorSubsystem));
    }
    return null;
  }

  public void teleopInit() {
    new ZeroElevatorCommand(m_elevatorSubsystem);
  }

  private static double modifyAxis(double value) {
    return MathUtil.applyDeadband(value, 0.1);
  }
}
