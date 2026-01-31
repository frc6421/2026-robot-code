// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  ClimbSubsystem climbSubsystem;
  double setPosition;
  /** Creates a new ClimbCommand. */
  public ClimbCommand(ClimbSubsystem climbSubsystem, double position) {
    this.climbSubsystem = climbSubsystem;
    setPosition = position;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.climbMotor.setControl(climbSubsystem.climbRequest.withPosition(setPosition / ClimbConstants.CLIMBER_INCHES_PER_ROTATION));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimbMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbSubsystem.withinError(setPosition);
  }
}
