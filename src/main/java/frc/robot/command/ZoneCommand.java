// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.rmi.server.ServerCloneException;
import java.util.concurrent.TransferQueue;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Zones;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZoneCommand extends Command {

  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  Supplier<SwerveDriveState> currentState;
  IntakeSubsystem intakeSubsystem;
  /** Creates a new ZoneCommand. */
  public ZoneCommand(
    IntakeSubsystem intakeSubsystem,
    TransitionSubsystem transitionSubsystem,
    ShooterSubsystem shooterSubsystem, Supplier<SwerveDriveState> currentState) {
    addRequirements(intakeSubsystem, transitionSubsystem, shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    this.currentState = currentState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(Zones.allianceZone.isPointInZone(
    //   currentState.get().Pose) + "::" + 
    //   (Zones.neutralZone.isPointInZone(currentState.get().Pose) || Zones.opposingZone.isPointInZone(currentState.get().Pose)));
    if (Zones.allianceZone.isPointInZone(currentState.get().Pose)) {
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateSOTM(currentState.get().Pose, currentState.get().Speeds);
      transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED,
        TransitionConstants.TRANSITION_HOPPER_SPEED);
      
    }
    else if (Zones.neutralHighZone.isPointInZone(currentState.get().Pose) || Zones.opposingZoneHigh.isPointInZone(currentState.get().Pose)){
      System.out.println("neutral high");
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateHighPOTM(currentState.get().Pose, currentState.get().Speeds);
      transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED,
        TransitionConstants.TRANSITION_HOPPER_SPEED);
    }

    else if (Zones.neutralLowZone.isPointInZone(currentState.get().Pose) || Zones.opposingZoneLow.isPointInZone(currentState.get().Pose)){
      System.out.println("neutral low");
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateLowPOTM(currentState.get().Pose, currentState.get().Speeds);
      transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED,
        TransitionConstants.TRANSITION_HOPPER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    intakeSubsystem.stopIntakePivot();
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
