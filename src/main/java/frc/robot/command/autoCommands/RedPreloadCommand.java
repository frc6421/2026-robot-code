
package frc.robot.command.autoCommands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.command.ShooterRevUp;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class RedPreloadCommand extends SequentialCommandGroup {
  /** Creates a new RedPreload. */
  public RedPreloadCommand(ClimbSubsystem climbSubsystem, CommandSwerveDrivetrain driveSubsystem,
  ShooterSubsystem shooterSubsystem, TransitionSubsystem transitionSubsystem) {
    
    addCommands(
      driveSubsystem.alignCommand(() -> Constants.TrajectoryConstants.RED_ALLIANCE),
      new ShooterRevUp(shooterSubsystem, transitionSubsystem, Constants.ShooterConstants.SHOOTER_RPM),
      transitionSubsystem.shooterTransition(
        Constants.TransitionConstants.TRANSITION_SHOOTER_SPEED,
        Constants.TransitionConstants.TRANSITION_HOPPER_SPEED),
      new WaitCommand(3.5),
      new InstantCommand(() -> transitionSubsystem.stopTransition()),
      new InstantCommand(() -> shooterSubsystem.stopShooter())
    );
  }
}