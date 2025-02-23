/*
 * Copyright (C) 2025 FRC Team 3602. Some rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team3602.robot.subsystems.ElevatorSubsystem;

import static frc.team3602.robot.Constants.OperatorInterfaces.*;

public class RobotContainer {
  /* Subsystems */
  private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();

  /* Superstructure */
  private final Superstructure superstructure = new Superstructure(elevatorSubsys);

  /* Operator interfaces */
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);

  public RobotContainer() {
    configButtonBindings();
  }

  /**
   * Function that is called in the constructor where we configure operator
   * interface button bindings.
   */
  private void configButtonBindings() {
    /* Elevator button bindings */
    xboxController.a().whileTrue(superstructure.coralIntake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
