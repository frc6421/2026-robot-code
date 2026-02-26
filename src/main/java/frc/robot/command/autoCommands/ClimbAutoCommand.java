// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.ShooterRevUp;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAutoCommand extends SequentialCommandGroup {
  /** Creates a new ClimbCommand. */
  public ClimbAutoCommand(ClimbSubsystem climb, CommandSwerveDrivetrain drive,
  ShooterSubsystem shooter, TransitionSubsystem transition) {
    addCommands(
      drive.profiledAlignCommand(() -> TrajectoryConstants.RED_CLIMB_DEPOT),
      drive.profiledAlignCommand(() -> TrajectoryConstants.RED_CLIMB_DEPOT_OFFSET)
    );
  }
}
