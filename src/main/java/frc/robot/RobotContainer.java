// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.command.ClimbCommand;
import frc.robot.command.FixedCommand;
import frc.robot.command.ShooterRevUp;
import frc.robot.command.TargetCommand;
import frc.robot.command.ZoneCommand;
import frc.robot.command.autoCommands.BlueDBackCommand;
import frc.robot.command.autoCommands.BlueDepotCommand;
import frc.robot.command.autoCommands.BlueOBackCommand;
import frc.robot.command.autoCommands.BluePreCommand;
import frc.robot.command.autoCommands.RedDBackCommand;
import frc.robot.command.autoCommands.RedDepotCommand;
import frc.robot.command.autoCommands.RedOBackCommand;
import frc.robot.command.autoCommands.RedPreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.net.CookieStore;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  // The robot's subsystems and commands are defined here...

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ClimbSubsystem climberSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TransitionSubsystem transitionSubsystem = new TransitionSubsystem();

  private final ClimbCommand climbGoesUpCommand = new ClimbCommand(climberSubsystem, Constants.ClimbPositions.L1_INCHES);
  private final ClimbCommand climbGoesDownCommand = new ClimbCommand(climberSubsystem, Constants.ClimbPositions.MATCH_START_INCHES);
  private final ShooterRevUp shootingRevUp = new ShooterRevUp(shooterSubsystem, transitionSubsystem, Constants.ShooterConstants.SHOOTER_RPM);
  private final ZoneCommand zoneCommand = new ZoneCommand(intakeSubsystem, transitionSubsystem, shooterSubsystem, () -> drivetrain.getState());
  private final TargetCommand targetCommand = new TargetCommand(intakeSubsystem, transitionSubsystem, shooterSubsystem, () -> drivetrain.getState());
  private final FixedCommand fixedCommand = new FixedCommand(intakeSubsystem, transitionSubsystem, shooterSubsystem, () -> drivetrain.getState());

  private final RedDBackCommand redDBackCommand = new RedDBackCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final BlueDBackCommand blueDBackCommand = new BlueDBackCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final BlueOBackCommand blueOBackCommand = new BlueOBackCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final BluePreCommand bluePreCommand = new BluePreCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final RedOBackCommand redOBackCommand = new RedOBackCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final RedPreCommand redPreCommand = new RedPreCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final RedDepotCommand redDepotCommand = new RedDepotCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final BlueDepotCommand blueDepotCommand = new BlueDepotCommand(intakeSubsystem, climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Double> hoodChooser;

  private final SlewRateLimiter xDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);
	private final SlewRateLimiter yDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Blue D Back", blueDBackCommand);
    autoChooser.addOption("Blue O Back", blueOBackCommand);
    autoChooser.addOption("Blue Pre", bluePreCommand);
    autoChooser.addOption("Blue Depot", blueDepotCommand);
    autoChooser.addOption("Red D Back", redDBackCommand);
    autoChooser.addOption("Red O Back", redOBackCommand);
    autoChooser.addOption("Red Depot", redDepotCommand);

    autoChooser.setDefaultOption("Red Pre", redPreCommand);

    hoodChooser = new SendableChooser<>();
    hoodChooser.addOption("Pass", Constants.ShooterConstants.ACTUATOR_PASSING);
    hoodChooser.setDefaultOption("Shoot", Constants.ShooterConstants.ACTUATOR_SHOOTING);
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

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Hood Chooser", hoodChooser);
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

    drivetrain.registerTelemetry(logger::telemeterize);

  joystick.back().onTrue(new InstantCommand(() -> drivetrain.visionGyroReset()));
  joystick.start().onTrue(drivetrain.resetGyro());
  // joystick.x().toggleOnTrue(zoneCommand);
  joystick.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.intakeOut()));
  joystick.leftBumper().onTrue(new InstantCommand(() -> intakeSubsystem.stopIntake()));
  joystick.y().onTrue(climbGoesUpCommand);
  joystick.a().onTrue(climbGoesDownCommand);
  joystick.b().onTrue(new InstantCommand(() -> intakeSubsystem.intakeIn()));
  joystick.x().onTrue(intakeSubsystem.setIntakeSpeed(-IntakePositions.INTAKE_SPEED));

  joystick.rightBumper().whileTrue(targetCommand);

  joystick.rightTrigger().whileTrue(fixedCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public double getHoodSelected() {
    return hoodChooser.getSelected();
  }

  public void autoVisionGyroReset() {
    drivetrain.visionGyroReset();
  }

  public void visionGyroResetBack() {
    drivetrain.visionGyroResetBackCam();
  }

  // @Override
  // public void simulationPeriodic() {
  //   drivetrain.getVisionSim().update(drivetrain.getState().Pose);
  // }


  public static void applyTalonConfigs(TalonFX motor, TalonFXConfiguration config) {
		StatusCode status = StatusCode.StatusCodeNotInitialized;
		for (int i = 0; i < 5; ++i) {
			status = motor.getConfigurator().apply(config);
			if (status.isOK())
				break;
		}
		if (!status.isOK()) {
			DataLogManager.log("Erorr:" + motor.getDescription() + " Configuration not applied " + status.toString());
		}
	}

}
