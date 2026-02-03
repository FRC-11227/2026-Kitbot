// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Autos2;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final BallSubsystem m_BallSubsystem = new BallSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData(m_DrivetrainSubsystem);
    SmartDashboard.putData(m_BallSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_shooterSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_shooterSubsystem));

    // Schedule `m_BallSubsystem.shootSequence()` when the Xbox controller's right bumper is pressed,
    // schedule ` m_BallSubsystem.stop()` when released
    m_driverController.rightBumper().whileTrue(m_BallSubsystem.shootSequence());

    // Continuously schedule `m_BallSubsystem.intake()` while the Xbox controller's left bumper is pressed,
    // schedule ` m_BallSubsystem.stop()` when released
    m_driverController.leftBumper()
        .whileTrue(m_BallSubsystem.runEnd(() -> m_BallSubsystem.intake(), () -> m_BallSubsystem.stop()));

    // Continuously schedule `m_BallSubsystem.eject()` while the Xbox controller's A button is pressed,
    // schedule ` m_BallSubsystem.stop()` when released.
    m_driverController.a()
        .whileTrue(m_BallSubsystem.runEnd(() -> m_BallSubsystem.eject(), () -> m_BallSubsystem.stop()));

    // When no other command is being scheduled for m_DrivetrainSubsystem, run m_DrivetrainSubsystem.driveArcade
    // with values from the left joystick
    m_DrivetrainSubsystem.setDefaultCommand(
        m_DrivetrainSubsystem.driveArcade(m_driverController::getLeftY, m_driverController::getLeftX)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.shootAndBackup(m_DrivetrainSubsystem, m_BallSubsystem);
    return Autos2.aftershootAndBackup(m_DrivetrainSubsystem, m_BallSubsystem);
  }
}
