// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
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
public class RedPreCommand extends SequentialCommandGroup {
  /** Creates a new RedPreCommand. */
 private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RedPreCommand(IntakeSubsystem intake, ClimbSubsystem climb, CommandSwerveDrivetrain drive,
  ShooterSubsystem shooter, TransitionSubsystem transition) {

    addCommands(
        //drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_PRE_SCORE),
        new ZoneCommand(intake, transition, shooter, () -> drive.getState())
    );
  }
}
