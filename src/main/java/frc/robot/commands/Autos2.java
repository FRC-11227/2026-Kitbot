// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.BallConstants;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos2 {
  public static Command driveAndTurn(DrivetrainSubsystem driveSubsystem) {
    return Commands.sequence(
        // Drive forward at 50% speed for 2 seconds
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
        // Stop for 1 second
        driveSubsystem.stop().withTimeout(1),
        // Spin left at 30% speed for 1 second
        driveSubsystem.driveArcade(() -> 0, () -> 0.3).withTimeout(1)
    );
  }


  public static Command aftershootAndBackup(DrivetrainSubsystem driveSubsystem, BallSubsystem ballSubsystem) {
    return Commands.sequence(
      
      driveSubsystem.driveArcade(() -> 0.25, () -> 0).withTimeout(5),

      driveSubsystem.driveArcade(() -> 0, () -> 0.25).withTimeout(5),
      
      ballSubsystem.shootSequence().withTimeout(BallConstants.SPIN_UP_SECONDS + 5),

      driveSubsystem.driveArcade(() -> -0.1, () -> 0).withTimeout(2.5),

      driveSubsystem.driveArcade(() -> 0, () -> -0.1).withTimeout(7),

      ballSubsystem.shootSequence().withTimeout(BallConstants.SPIN_UP_SECONDS + 5),
      
      driveSubsystem.stop()
    );
  }

  private Autos2() {
    throw new UnsupportedOperationException("Skill Issue!");
  }
}
