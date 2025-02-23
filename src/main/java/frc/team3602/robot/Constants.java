/*
 * Copyright (C) 2025 FRC Team 3602. Some rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public final class Constants {
  public final class OperatorInterfaces {
    public final static int kXboxControllerPort = 1;
  }

  public final class Elevator {
    public final static Time kLoopTime = Seconds.of(0.020);

    /* Motor CAN IDs */
    public final static int kLeaderMotorID = 2;
    public final static int kFollowerMotorID = 3;

    /* Encoder channels */
    public final static int kEncoderChannelA = 0;
    public final static int kEncoderChannelB = 1;

    /* Elevator scoring heights */
    public final static Distance kCoralIntake = Inches.of(0.0);

    /* Motion profile */
    public final static LinearVelocity kMaxVelocity = MetersPerSecond.of(0.0);
    public final static LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(0.0);

    /* Linear system */
    public final static Mass kMass = Kilograms.of(0.0);
    public final static Distance kRadius = Meters.of(0.0);
    public final static double kGearing = (12.0 / 1.0);

    /* Kalman filter */
    public final static Distance kPositionDeviation = Meters.of(0.0);
    public final static LinearVelocity kVelocityDeviation = MetersPerSecond.of(0.0);
    public final static double kEncoderDeviation = 0.001;

    /* Linear quadratic regulator */
    public final static Distance kPositionTolerance = Meters.of(0.0);
    public final static LinearVelocity kVelocityTolerance = MetersPerSecond.of(0.0);
    public final static Voltage kControlEffortTolerance = Volts.of(12.0);

    /* Linear system loop */
    public final static Voltage kMaxVoltage = Volts.of(12.0);
  }
}
