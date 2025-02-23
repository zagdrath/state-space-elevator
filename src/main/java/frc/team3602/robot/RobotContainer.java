/*
 * Copyright (C) 2025 FRC Team 3602. Some rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  /* Subsystems */
  private final static ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
