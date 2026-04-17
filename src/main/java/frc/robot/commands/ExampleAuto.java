// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // Drive for 1 seconds. The driveArcadeAuto command factory
    // intentionally creates a command which does not end which allows us to control
    // the timing using the withTimeout decorator
    new AutoDrive(driveSubsystem,0.5,  0.0).withTimeout(1),

        // Spin up the launcher and then launch balls
    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5),

    new LaunchSequence(ballSubsystem).withTimeout(1.5),
    new WaitCommand(.5)

    );

    


  }
}
