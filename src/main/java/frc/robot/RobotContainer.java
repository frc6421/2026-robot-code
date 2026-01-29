// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  // The robot's subsystems and commands are defined here...

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SlewRateLimiter xDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);
	private final SlewRateLimiter yDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

/**
 * align to climb: while true, a
 * l1 climb: on true, y
 * turn to goal: while true, right bumper
 * shoot: while true, right trigger
 * intake on: on true, x (closest to joystick)
 * intake off: on true, b
 * pass align: while true, left bumper
 * pass: while true, left trigger
 * 
 * in future, combine aligns with shots
 */

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
    drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() -> drive
					// Drive forward with negative Y (forward)
					.withVelocityX(xDriveSlew.calculate(-joystick.getLeftY() * MaxSpeed))
					// Drive left with negative X (left)
					.withVelocityY(yDriveSlew.calculate(-joystick.getLeftX() * MaxSpeed))
					// Drive counterclockwise with negative X (left)
					.withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

     joystick.a().onTrue(new InstantCommand(() -> SignalLogger.start()));
		 joystick.b().onTrue(new InstantCommand(() -> SignalLogger.stop()));
     joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		 joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
		 joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
     joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
