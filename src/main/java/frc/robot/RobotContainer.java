// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("CenterAuto");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new Intake(fuelSubsystem).withTimeout(1.0));
    NamedCommands.registerCommand("Eject", new Eject(fuelSubsystem).withTimeout(0.75));
    NamedCommands.registerCommand("LaunchSequence", new LaunchSequence(fuelSubsystem).withTimeout(2.0));
  }

  private void configureBindings() {
    driverController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    driverController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    driverController.a().whileTrue(new Eject(fuelSubsystem));
    driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}