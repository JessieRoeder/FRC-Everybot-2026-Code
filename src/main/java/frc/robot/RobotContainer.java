package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.TestAuto;
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

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Example Auto", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Autonomous 2", new TestAuto(driveSubsystem, fuelSubsystem));
    SmartDashboard.putData(autoChooser);
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

    // SysId rotation tests for MOI characterization.
    // Run these in Test mode and hold the buttons.
    operatorController.a().whileTrue(
        driveSubsystem.sysIdRotationQuasistatic(SysIdRoutine.Direction.kForward));
    operatorController.b().whileTrue(
        driveSubsystem.sysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse));
    operatorController.x().whileTrue(
        driveSubsystem.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
    operatorController.y().whileTrue(
        driveSubsystem.sysIdRotationDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}