// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.ClimbAuto;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.Turn90;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(m_turretSubsystem);

  private final CommandXboxController m_primaryController = new CommandXboxController(0);

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
      m_primaryController.getLeftY() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      m_primaryController.getRightX() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      true),
      m_driveSubsystem));
  
    m_autoChooser.setDefaultOption("Do Nothing", () -> new DriveUntilCommand(m_driveSubsystem, 0, () -> true));

    m_driveSubsystem.followTrajectoryCommand("Test", true);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    NamedCommands.registerCommand("Turn 90", new Turn90(m_driveSubsystem, true));
    
    configureBindings();
  }

  private void configureBindings() {

    m_primaryController.leftStick().and(m_primaryController.rightStick())
      .onTrue(new ClimbAuto(m_driveSubsystem, 1));
      
  }

  public Command getAutonomousCommand() {
    var autoCommandSupplier = m_autoChooser.getSelected();
    if (autoCommandSupplier != null) {
      return autoCommandSupplier.get()
        .beforeStarting(() -> m_turretSubsystem.zeroTurret(), m_turretSubsystem)
        .beforeStarting(() -> m_elevatorSubsystem.zeroElevator(), m_elevatorSubsystem);
    }
    return null;
  }

  public void teleopInit() {
    m_turretSubsystem.zeroTurret();
    m_elevatorSubsystem.zeroElevator();
  }
}
