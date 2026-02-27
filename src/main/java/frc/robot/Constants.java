// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANConstants {
    // CAN IDs of all devices in system
    public static final int DRIVETRAIN_LEFT_LEAD = 1;
    public static final int DRIVETRAIN_LEFT_FOLLOW = 2;
    public static final int DRIVETRAIN_RIGHT_LEAD = 3;
    public static final int DRIVETRAIN_RIGHT_FOLLOW = 4;
    public static final int INTAKE_SHOOTER_MOTOR = 5;
    public static final int FEEDER_MOTOR = 6;
  }

  public static class MotorConstants {
    public static final int CIM_CURRENT_LIMIT = 60;
    public static final int NEO_CURRENT_LIMIT = 40;
  }

  public static class DriveConstants {
    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = MotorConstants.CIM_CURRENT_LIMIT;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These values MUST be determined either experimentally or theoretically for *your* robot's
    // drive. The SysId tool provides a convenient method for obtaining feedback and feedforward
    // values for your robot.
    public static final double kTurnP = 0.05;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerDegree = 0.8;
    public static final double kaVoltSecondsSquaredPerDegree = 0.15;
  }

  public static class BallConstants {
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = MotorConstants.NEO_CURRENT_LIMIT;
    public static final int SHOOTER_MOTOR_CURRENT_LIMIT = MotorConstants.NEO_CURRENT_LIMIT;

    public static final double DEFAULT_INTAKING_FEEDER_SPEED = -1;
    public static final double DEFAULT_INTAKING_INTAKE_SPEED = 0.83;
    public static final double DEFAULT_SPINUP_FEEDER_SPEED = -0.5;
    public static final double DEFAULT_SHOOTING_FEEDER_SPEED = 0.75;
    public static final double DEFAULT_SHOOTING_SHOOTER_SPEED =0.88;
    public static final double SPIN_UP_SECONDS = 1; // Will be replaced with PID in the future
  }
}
