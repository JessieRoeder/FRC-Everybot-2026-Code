&#x20;   // While the **left bumper** is **held**, **intake Fuel**

&#x20;   driverController.leftBumper().whileTrue(new Intake(fuelSubsystem));



&#x20;   // While the **right bumper** is **held**, **spin up for 1**

&#x20;   **// second, then launch fuel**. When the button is **released, stop**.

&#x20;   driverController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));



&#x20;   // While the **A button** is **held**, eject fuel back out

&#x20;   // the intake

&#x20;   driverController.a().whileTrue(new Eject(fuelSubsystem));



&#x20;  // While the **down arrow** on the directional pad is **held** it will **un-climb** the robot

&#x20;   driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));



&#x20;   // While the **up arrow** on the directional pad is **held** it will **climb** the robot

&#x20;   driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

