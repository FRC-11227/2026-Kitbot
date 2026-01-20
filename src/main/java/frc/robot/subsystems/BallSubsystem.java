package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallConstants;
import frc.robot.Constants.CANConstants;

public class BallSubsystem extends SubsystemBase {
    private final SparkMax intakeShooterMotor;
    private final SparkMax feederMotor;
    
    /** Create a new BallSubsystem */
    public BallSubsystem() {
        intakeShooterMotor = new SparkMax(CANConstants.INTAKE_SHOOTER_MOTOR, MotorType.kBrushed);
        feederMotor = new SparkMax(CANConstants.FEEDER_MOTOR, MotorType.kBrushed);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
            .smartCurrentLimit(BallConstants.FEEDER_MOTOR_CURRENT_LIMIT)
            .voltageCompensation(12);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeShooterConfig = new SparkMaxConfig();
        feederConfig
            .smartCurrentLimit(BallConstants.SHOOTER_MOTOR_CURRENT_LIMIT)
            .voltageCompensation(12)
            .inverted(true);
        intakeShooterMotor.configure(intakeShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Feeder speed when intaking", BallConstants.DEFAULT_INTAKING_FEEDER_SPEED);
        SmartDashboard.putNumber("Intake speed when intaking", BallConstants.DEFAULT_INTAKING_INTAKE_SPEED);
        SmartDashboard.putNumber("Feeder speed while shooter spins up", BallConstants.DEFAULT_SPINUP_FEEDER_SPEED);
        SmartDashboard.putNumber("Feeder speed when shooting", BallConstants.DEFAULT_SHOOTING_FEEDER_SPEED);
        SmartDashboard.putNumber("Shooter speed when shooting", BallConstants.DEFAULT_SHOOTING_SHOOTER_SPEED);
    }

    public void intake() {
        feederMotor.set(SmartDashboard.getNumber("Feeder speed when intaking", BallConstants.DEFAULT_INTAKING_FEEDER_SPEED));
        intakeShooterMotor.set(SmartDashboard.getNumber("Intake speed when intaking", BallConstants.DEFAULT_INTAKING_INTAKE_SPEED));
    }

    public void eject() {
        feederMotor.set(-1 * SmartDashboard.getNumber("Feeder speed when intaking", BallConstants.DEFAULT_INTAKING_FEEDER_SPEED));
        intakeShooterMotor.set(-1 * SmartDashboard.getNumber("Intake speed when intaking", BallConstants.DEFAULT_INTAKING_INTAKE_SPEED));
    }

    public void spinUpShooter() {
        feederMotor.set(SmartDashboard.getNumber("Feeder speed while shooter spins up", BallConstants.DEFAULT_SPINUP_FEEDER_SPEED));
        intakeShooterMotor.set(SmartDashboard.getNumber("Shooter speed when shooting", BallConstants.DEFAULT_SHOOTING_SHOOTER_SPEED));
    }

    public void shoot() {
        feederMotor.set(SmartDashboard.getNumber("Feeder speed when shooting", BallConstants.DEFAULT_SHOOTING_FEEDER_SPEED));
        intakeShooterMotor.set(SmartDashboard.getNumber("Shooter speed when shooting", BallConstants.DEFAULT_SHOOTING_SHOOTER_SPEED));
    }

    public Command spinUpCommand() {
        return this.run(() -> spinUpShooter());
    }

    public Command shootCommand() {
        return this.run(() -> shoot());
    }

    public void stop() {
        feederMotor.set(0);
        intakeShooterMotor.set(0);
    }

    public Command shootSequence() {
        return spinUpCommand()
            .withTimeout(BallConstants.SPIN_UP_SECONDS)
            .andThen(shootCommand())
            .finallyDo(() -> stop());
    }
}
