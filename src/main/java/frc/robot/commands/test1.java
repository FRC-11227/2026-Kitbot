// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.Constants.BallConstants;
// import frc.robot.subsystems.BallSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import java.awt.event.ActionEvent;
// import java.awt.event.ActionListener;

// private final class teleoplearningtest {
//    <abutton>
  
//   public static final int CONTROLLER_PORT = 0;
//   public XboxController controller; 


//   JButton button = new JButton
//   if <a button> pressed {
//     ballSubsystem.shootSequence().withTimeout(BallConstants.SPIN_UP_SECONDS + 6)

//   }
// } 





// public final class autotest2 { 
//   public static Command driveAndTurn(DrivetrainSubsystem driveSubsystem) {
//     return Commands.sequence(

//       driveSubsystem.driveArcade(() -> 0.15, () -> 0).withTimeout(2),

//       driveSubsystem.stop().withTimeout(0.5),

//       driveSubsystem.driveArcade(() -> 0,  ()-> 0.25).withTimeout(1)
   
//     );
//   }

//   public static Command Shoot(DrivetrainSubsystem driveSubsytem, BallSubsystem ballSubsystem) {
//     return Commands.sequence(
//       ballSubsystem.shootSequence().withTimeout(BallConstants.SPIN_UP_SECONDS + 6),
//       driveSubsytem.stop()
//     );
//   } 
// }