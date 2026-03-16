// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.TargetCommand;
import frc.robot.command.ZoneCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedOBackCommand extends SequentialCommandGroup {
  /** Creates a new RedOBackCommand. */
   private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RedOBackCommand(IntakeSubsystem intake, ClimbSubsystem climb, CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter, TransitionSubsystem transition) {

    addCommands(
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_OUTPOST_SCORE),
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RO),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RO_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RO_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_OUTPOST_SCORE),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RO),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RO_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RO_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_OUTPOST_SCORE),
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
