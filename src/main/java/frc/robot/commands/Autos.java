// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  public static Command driveAndTurn(DrivetrainSubsystem drive) {
    return Commands.sequence(
        // Drive forward at 50% speed for 2 seconds
        drive.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
        // Stop for 1 second
        drive.stopDrive().withTimeout(1),
        // Spin left at 30% speed for 1 second
        drive.driveArcade(() -> 0, () -> 0.3).withTimeout(1)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
