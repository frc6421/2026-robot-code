// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimbPositions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.ClimbCommand;
import frc.robot.command.ShooterRevUp;
import frc.robot.command.ZoneCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedPreClimbCommand extends SequentialCommandGroup {
  /** Creates a new RedPreClimbCommand. */

  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RedPreClimbCommand(IntakeSubsystem intake, ClimbSubsystem climb, CommandSwerveDrivetrain drive,
  ShooterSubsystem shooter, TransitionSubsystem transition) {

    addCommands(
      drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_SCORE),
      new ParallelDeadlineGroup(
          new WaitCommand(3),
          new ZoneCommand(intake, transition, shooter, () -> drive.getState())
          ),
      drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_CLIMB_DEPOT_OFFSET), 
      new InstantCommand(() -> shooter.setRPM(0)),
      drive.profiledAlignCommand(() -> TrajectoryConstants.RED_CLIMB_DEPOT),
      new ClimbCommand(climb, ClimbPositions.L1_INCHES), 
      drive.profiledAlignCommand(() -> TrajectoryConstants.RED_CLIMB_DEPOT_FINAL),
      new ClimbCommand(climb, ClimbPositions.MATCH_START_INCHES)
    );
  }
}
