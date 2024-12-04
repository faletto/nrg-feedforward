// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class Feedforward extends Command {
  private double kS;
  private double kV;
  private double kA;

  private double targetVelocity;
  private final SwerveSubsystem swerve;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  /** Creates a new FeedforwardFeedback. */
  public Feedforward(SwerveSubsystem swerve, double targetVelocity, double kS, double kV, double kA) {
    this.swerve = swerve;
    this.targetVelocity = targetVelocity;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetPosition(new Pose2d());
    swerve.resetOrientation(new Rotation2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedForwardOutput = feedforward.calculate(targetVelocity);
    swerve.drive(feedForwardOutput, 0,0,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
