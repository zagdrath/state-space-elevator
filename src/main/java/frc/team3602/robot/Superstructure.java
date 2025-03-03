/*
 * Copyright (C) 2025 FRC Team 3602. Some rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.ElevatorSubsystem;

import static frc.team3602.robot.Constants.Elevator.*;

public class Superstructure {
  /* Subsystems */
  private ElevatorSubsystem elevatorSubsys;

  public Superstructure(ElevatorSubsystem elevatorSubsys) {
    this.elevatorSubsys = elevatorSubsys;
  }

  public Command coralIntake() {
    return Commands.sequence(
      elevatorSubsys.setElevatorState(kCoralIntake)
    );
  }
}
