// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Intake intake = Intake.getInstance(); 
  private CommandXboxController driverController = new CommandXboxController(0);  
  private CommandXboxController manipulatorController = new CommandXboxController(1);  
  public RobotContainer() {
    configureSubsystems(); 
    configureBindings();
  }

  private void configureSubsystems() {
    drivetrain.setDefaultCommand(drivetrain.driveCommand(driverController.getHID()));
  }

  private void configureBindings() {
    // driver
    driverController.a().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));


    // manipulator
    manipulatorController.rightTrigger()
    .onFalse(
      intake.bottomSpeed(1)
        .andThen(new WaitCommand(0.5))
        .andThen(intake.stopAll())
    )
    .whileTrue(intake.topSpeed(1)); 
    manipulatorController.leftTrigger().onTrue(intake.intakeNote(0.5)).onFalse(intake.stopAll()); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
