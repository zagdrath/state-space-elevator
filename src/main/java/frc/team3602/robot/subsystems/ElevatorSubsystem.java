package frc.team3602.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.team3602.robot.Constants.Elevator.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorSubsystem extends SubsystemBase {
  /* Motors */
  private final TalonFX leaderMotor = new TalonFX(kLeaderMotorID);
  private final TalonFX followerMotor = new TalonFX(kFollowerMotorID);

  /* Encoders */
  private final Encoder elevatorEncoder = new Encoder(kEncoderChannelA, kEncoderChannelB);

  /* Controls */
  private TrapezoidProfile.State elevatorHeight;

  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      kMaxVelocity.in(MetersPerSecond), kMaxAcceleration.in(MetersPerSecondPerSecond)));
  private TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State();

  // The plant holds a state-space model of an elevator
  private final LinearSystem<N2, N1, N2> elevatorPlant = LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(2),
      kMass.in(Kilograms), kRadius.in(Meters), kGearing);

  // The observer fuses encoder data and voltage inputs to reject noise
  @SuppressWarnings("unchecked")
  private final KalmanFilter<N2, N1, N1> elevatorObserver = new KalmanFilter<>(Nat.N2(), Nat.N1(),
      (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
      VecBuilder.fill(kPositionDeviation.in(Meters), kVelocityDeviation.in(MetersPerSecond)),
      VecBuilder.fill(kEncoderDeviation), kLoopTime.in(Seconds));

  // The linear quadratic regulator uses feedback to create voltage commands
  @SuppressWarnings("unchecked")
  private final LinearQuadraticRegulator<N2, N1, N1> elevatorController = new LinearQuadraticRegulator<>(
      (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
      VecBuilder.fill(kPositionTolerance.in(Meters), kVelocityTolerance.in(MetersPerSecond)),
      VecBuilder.fill(kControlEffortTolerance.in(Volts)), kLoopTime.in(Seconds));

  // The state-space loop combines the controller, observer, feedforward, and
  // plant for easy control
  @SuppressWarnings("unchecked")
  private final LinearSystemLoop<N2, N1, N1> elevatorLoop = new LinearSystemLoop<>(
      (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
      elevatorController, elevatorObserver,
      kMaxVoltage.in(Volts), kLoopTime.in(Seconds));

  public ElevatorSubsystem() {
    // Reset the loop to make sure it's in a known state
    elevatorLoop.reset(VecBuilder.fill(getEncoderDistance(), getEncoderRate()));

    // Reset our last profiled reference to the current state
    lastProfiledReference = new TrapezoidProfile.State(getEncoderDistance(), getEncoderRate());

    configElevatorSubsys();
  }

  private double getEncoderDistance() {
    return elevatorEncoder.getDistance();
  }

  private double getEncoderRate() {
    return elevatorEncoder.getRate();
  }

  @Override
  public void periodic() {
    // Step the trapezoid profile forward 20ms and set it as the next reference
    lastProfiledReference = trapezoidProfile.calculate(kLoopTime.in(Seconds), lastProfiledReference, elevatorHeight);
    elevatorLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

    // Correct the kalman filter's state vector estimate with encoder data
    elevatorLoop.correct(VecBuilder.fill(getEncoderDistance()));

    // Update the linear quadration regulator to generate new voltage commands and
    // use the voltages to predict the next state with the kalman filter
    elevatorLoop.predict(kLoopTime.in(Seconds));

    // Send the new calculated voltage to the motors
    var nextVoltage = elevatorLoop.getU(0);
    leaderMotor.setVoltage(nextVoltage);
  }

  private void configElevatorSubsys() {
    /* Motor configs */
    var motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leaderMotor.getConfigurator().apply(motorOutputConfigs);

    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    followerMotor.getConfigurator().apply(motorOutputConfigs);

    // Ensure the follower is following the leader
    followerMotor.setControl(new Follower(kLeaderMotorID, false));

    /* Encoder configs */
    elevatorEncoder.setDistancePerPulse(0.0);
  }
}
