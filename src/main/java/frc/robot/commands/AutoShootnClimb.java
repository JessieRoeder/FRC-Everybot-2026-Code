// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootnClimb extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public AutoShootnClimb(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ClimbUp(climberSubsystem).withTimeout(3.5).alongWith(new AutoDrive(driveSubsystem,0.6,  0.0).withTimeout(1)),


    // Spin up the launcher and then launch balls
    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),



    new AutoDrive(driveSubsystem,0.5,  0.0).withTimeout(2),
    new ClimbDown(climberSubsystem).withTimeout(4)
    );

  }
}