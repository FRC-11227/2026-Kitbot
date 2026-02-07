// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class autotest2 { 
  public static Command driveAndTurn(DrivetrainSubsystem driveSubsystem) {
    return Commands.sequence(

      driveSubsystem.driveArcade(() -> 0.15, () -> 0).withTimeout(2),

      driveSubsystem.stop().withTimeout(0.5),

      driveSubsystem.driveArcade(() -> 0,  ()-> 0.25).withTimeout(0.5)
   
    );
  }
}