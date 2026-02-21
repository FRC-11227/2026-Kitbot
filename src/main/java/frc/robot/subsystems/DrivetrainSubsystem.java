// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SparkMax m_leftLead;
  private final SparkMax m_leftFollow;
  private final SparkMax m_rightLead;
  private final SparkMax m_rightFollow;

  private final DifferentialDrive m_drivetrain;

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      DriveConstants.kTurnP,
      DriveConstants.kTurnI,
      DriveConstants.kTurnD,
      new TrapezoidProfile.Constraints(
          DriveConstants.kMaxTurnRateDegPerS,
          DriveConstants.kMaxTurnAccelerationDegPerSSquared));
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerDegree,
      DriveConstants.kaVoltSecondsSquaredPerDegree);

  static final double kP = 0.03;

  /** Creates and configures a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_leftLead = new SparkMax(CANConstants.DRIVETRAIN_LEFT_LEAD, MotorType.kBrushed);
    m_leftFollow = new SparkMax(CANConstants.DRIVETRAIN_LEFT_FOLLOW, MotorType.kBrushed);
    m_rightLead = new SparkMax(CANConstants.DRIVETRAIN_RIGHT_LEAD, MotorType.kBrushed);
    m_rightFollow = new SparkMax(CANConstants.DRIVETRAIN_RIGHT_FOLLOW, MotorType.kBrushed);

    SparkMaxConfig config = new SparkMaxConfig();

    // Setup base config
    config
        .voltageCompensation(12)
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        .idleMode(SparkMaxConfig.IdleMode.kBrake);

    // Set leftFollow motor to follow the leftLead motor
    config.follow(m_leftLead);
    m_leftFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set rightFollow motor to follow the rightLead motor
    config.follow(m_rightLead);
    m_rightFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Disable following and apply config to leftLead motor
    config.disableFollowerMode();
    m_leftLead.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Invert control, and apply config to rightLead motor
    config.inverted(true);
    m_rightLead.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create a DifferentialDrive object using the left and right leader motors
    m_drivetrain = new DifferentialDrive(m_leftLead, m_rightLead);
  }

  /**
   * Stops the drivetrain and sets all motors to 0
   */
  public Command stop() {
    return this.run(() -> m_drivetrain.tankDrive(0, 0));
  }

  /**
   * Control the robot using an "Arcade Drive" style of control
   * 
   * @param speed    Forward-back speed of the robot (where -1.0 is full
   *                 backwards, and 1.0 is full forwards)
   * @param rotation Left-right speed of the robot (where -1.0 is full left, and
   *                 1.0 is full right)
   */
  public Command driveArcade(DoubleSupplier speed, DoubleSupplier rotation) {
    return this.run(() -> m_drivetrain.curvatureDrive(speed.getAsDouble(), rotation.getAsDouble(), true));
  }

  /**
   * Control the robot using a "Tank Drive" style of control
   * 
   * @param leftSpeed Speed for the left wheels of the robot (where -1.0 is full
   *                  backwards, and 1.0 is full forwards)
   * @param rotation  Speed for the right wheels of the robot (where -1.0 is full
   *                  left, and 1.0 is full right)
   */
  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    return this.run(() -> m_drivetrain.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble()));
  }

  public Command resetGyro() {
    return this.runOnce(() -> m_gyro.reset());
  }

  public Command rotateDegrees(Double setpoint) {
    return startRun(
        () -> {
          m_controller.reset(m_gyro.getRotation2d().getDegrees());
        },
        () -> {
          m_drivetrain.arcadeDrive(
            0,
            (m_controller.calculate(m_gyro.getRotation2d().getDegrees(), setpoint)
                // Divide feedforward voltage by battery voltage to normalize it to [-1, 1]
                + m_feedforward.calculate(m_controller.getSetpoint().velocity) / RobotController.getBatteryVoltage()) * -1);
          
          System.out.println(m_gyro.getRotation2d().getDegrees());
          }
        )

        .until(m_controller::atGoal)
        .finallyDo(() -> m_drivetrain.arcadeDrive(0, 0)).withName("Rotating robot");
  }

  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    double kP = .035;
    double kI = .000;
    double kD = .00001;
    double integral = 0;
    double prevError = 0;
    double error = LimelightHelpers.getTX("limelight");
    integral += error * 0.02; 
    double derivative = (error - prevError) / 0.02;
    double targetingAngularVelocity = (kP * error) + (kI * integral) + (kD * derivative);
    prevError = error;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Math.PI;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public Command rotateToTarget() {
    return run(() -> m_drivetrain.arcadeDrive(0, limelight_aim_proportional()));
  }

  public Command moveForward(Double speed) {
    return run(() -> m_drivetrain.arcadeDrive(speed, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
