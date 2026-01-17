// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class DriveForTimeChen extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final DrivetrainSubsystem m_drivetrain;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;
  private final double m_runTime;

  private final Timer m_Timer;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForTimeChen(DrivetrainSubsystem drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, double runTime) {
    m_drivetrain = drivetrain;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    m_runTime = runTime;

    m_Timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Timer.get() < m_runTime / 3) {
      m_drivetrain.tankDrive(m_leftSpeed, m_rightSpeed); // drive forwards
    } else if (m_Timer.get() > m_runTime / 3 && m_Timer.get() < m_runTime / 2) { 
      m_drivetrain.tankDrive(m_leftSpeed, () -> -m_rightSpeed.getAsDouble()); // rotate
    } else if (m_Timer.get() > m_runTime / 2) {
      m_drivetrain.tankDrive(() -> -m_leftSpeed.getAsDouble(), () -> -m_rightSpeed.getAsDouble()); // drive backwards
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(() -> 0, () -> 0);
    m_Timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > m_runTime;
  }
}
