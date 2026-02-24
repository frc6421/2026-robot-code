// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedDepotBumpCrossCommand extends SequentialCommandGroup {
  /** Creates a new RedDepotBumpCrossCommand. */
   public RedDepotBumpCrossCommand(ClimbSubsystem climb, CommandSwerveDrivetrain drive, IntakeSubsystem intake,
      ShooterSubsystem shooter, TransitionSubsystem transition) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        drive.alignCommand(() -> TrajectoryConstants.RED_BUMP_D_L),
        drive.alignCommand(() -> TrajectoryConstants.NEUTRAL_RD), 
        drive.alignCommand(() -> TrajectoryConstants.NEUTRAL_RO),
        drive.alignCommand(() -> TrajectoryConstants.RED_BUMP_O_R)

    );
  }
}
