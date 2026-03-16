// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FixedCommand extends Command {
  /** Creates a new FixedCommand. */
  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  Supplier<SwerveDriveState> currentState;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new TargetCommand. */
  public FixedCommand(
      IntakeSubsystem intakeSubsystem,
      TransitionSubsystem transitionSubsystem,
      ShooterSubsystem shooterSubsystem, Supplier<SwerveDriveState> currentState) {

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    this.currentState = currentState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("fixed :(");

    //shooterSubsystem.updateSOTM(currentState.get().Pose, currentState.get().Speeds);
    shooterSubsystem.turnShooter(0.0);
    shooterSubsystem.setRPM(2600);

    transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED,
        TransitionConstants.TRANSITION_HOPPER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transitionSubsystem.shooterTransition(
        0.0, 0.0);
    shooterSubsystem.stopShooter();
    shooterSubsystem.stopShooterTurn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
