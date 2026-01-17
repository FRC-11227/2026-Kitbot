// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SparkMax m_leftLead;
    private final SparkMax m_leftFollow;
    private final SparkMax m_rightLead;
    private final SparkMax m_rightFollow;

    private final DifferentialDrive m_drivetrain;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    m_leftLead = new SparkMax(frc.robot.Constants.CAN.DRIVETRAIN_LEFT_LEAD, MotorType.kBrushless);
    m_leftFollow = new SparkMax(frc.robot.Constants.CAN.DRIVETRAIN_LEFT_FOLLOW, MotorType.kBrushless);
    m_rightLead = new SparkMax(frc.robot.Constants.CAN.DRIVETRAIN_RIGHT_LEAD, MotorType.kBrushless);
    m_rightFollow = new SparkMax(frc.robot.Constants.CAN.DRIVETRAIN_RIGHT_FOLLOW, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40);

    leftLeadConfig
        .apply(globalConfig);
    
    leftFollowerConfig
        .apply(globalConfig)
        .follow(m_leftLead);

    rightLeadConfig
        .apply(globalConfig)
        .inverted(true);

    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLead);

    m_leftLead.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollow.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLead.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollow.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drivetrain = new DifferentialDrive(m_leftLead, m_rightLead);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  public void drive(double speed, double rotation) {
    m_drivetrain.curvatureDrive(speed, rotation, true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(leftSpeed, rightSpeed);
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
