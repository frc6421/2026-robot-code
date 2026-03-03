// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.command.ClimbCommand;
import frc.robot.command.ShooterRevUp;
import frc.robot.command.ZoneCommand;
import frc.robot.command.autoCommands.BluePreloadCommand;
import frc.robot.command.autoCommands.ClimbAutoCommand;
import frc.robot.command.autoCommands.RedPreloadCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final BluePreloadCommand bluePreloadCommand = new BluePreloadCommand(climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final RedPreloadCommand redPreloadCommand = new RedPreloadCommand(climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);
  private final ClimbAutoCommand climbAutoCommand = new ClimbAutoCommand(climberSubsystem, drivetrain, shooterSubsystem, transitionSubsystem);


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
    autoChooser.addOption("Red Preload", redPreloadCommand);
    autoChooser.addOption("Blue Preload", bluePreloadCommand);
    autoChooser.addOption("Climb Auto", climbAutoCommand);

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

    //  joystick.a().onTrue(new InstantCommand(() -> SignalLogger.start()));
		//  joystick.b().onTrue(new InstantCommand(() -> SignalLogger.stop()));
    //  joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		//  joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
		//  joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //  joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.x().onTrue(climbDownCommand);
    // joystick.y().onTrue(climbUpCommand);

    // joystick.a().onTrue(new InstantCommand(() -> intakeSubsystem.turnIntake(45)));

    drivetrain.registerTelemetry(logger::telemeterize);

    // transitionSubsystem.setDefaultCommand(transitionSubsystem.shooterTransition(
    //   TransitionConstants.TRANSITION_SHOOTER_SPEED, TransitionConstants.TRANSITION_HOPPER_SPEED));
    // intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeOut());
    // joystick.x().onTrue(new InstantCommand(() -> shooterSubsystem.turnShooter(45)));
    // joystick.y().onTrue(new InstantCommand(() -> shooterSubsystem.turnShooter(0.0)));

    // joystick.a().whileTrue(new InstantCommand(() -> shooterSubsystem.extendActuator(0.85)));
    // joystick.b().whileTrue(new InstantCommand(() -> shooterSubsystem.extendActuator(0.10)));

    // joystick.a().whileTrue(intakeSubsystem.setIntakePivotSpeed(0.25));
    // joystick.a().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntakePivot()));

    // joystick.b().whileTrue(intakeSubsystem.setIntakePivotSpeed(-0.25));
    // joystick.b().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntakePivot()));
    //intake
  //   joystick.x().onTrue(new SequentialCommandGroup(
  //     intakeSubsystem.setIntakeSpeed(.72),
  //     intakeSubsystem.setIntakePivotSpeed(.5),
  //     transitionSubsystem.intakeTransition(Constants.TransitionConstants.TRANSITION_HOPPER_SPEED)));
  //   joystick.b().onTrue(new SequentialCommandGroup(
  //     new InstantCommand(() -> intakeSubsystem.stopIntakePivot()),
  //     new InstantCommand(() -> intakeSubsystem.stopIntake()),
  //     new InstantCommand(() -> transitionSubsystem.stopTransition())
  //   ));

  //   //barf
  //   joystick.rightBumper().whileTrue(
  //     transitionSubsystem.shooterTransition(
  //       Constants.TransitionConstants.TRANSITION_SHOOTER_SPEED,
  //       Constants.TransitionConstants.TRANSITION_HOPPER_SPEED));
  //   joystick.rightBumper().onFalse(new InstantCommand(() -> transitionSubsystem.stopTransition()));
  
  // //shooting
  // joystick.rightTrigger().whileTrue(new SequentialCommandGroup(
  //   shootingRevUp,
  //   new InstantCommand(() -> shooterSubsystem.extendActuator(Constants.ShooterConstants.ACTUATOR_SHOOTING)),
  //   transitionSubsystem.shooterTransition(
  //     Constants.TransitionConstants.TRANSITION_SHOOTER_SPEED,
  //     Constants.TransitionConstants.TRANSITION_HOPPER_SPEED)));
  // joystick.rightTrigger().onFalse(new ParallelCommandGroup(
  //   new InstantCommand(() -> transitionSubsystem.stopTransition()),
  //   new InstantCommand(() -> shooterSubsystem.stopShooter())));

  // joystick.leftBumper().whileTrue(drivetrain.profiledAlignCommand( 
  //     () -> DriverStation.getAlliance()
  //     .map(alliance ->
  //       alliance == Alliance.Blue
  //       ? TrajectoryConstants.BLUE_ALLIANCE
  //       : TrajectoryConstants.RED_CLIMB_DEPOT
  // )
  // .orElse(TrajectoryConstants.BLUE_ALLIANCE)
  // ));
  // // .get() == Alliance.Blue) ? 
  // // TrajectoryConstants.BLUE_ALLIANCE :
  // // TrajectoryConstants.RED_ALLIANCE));


  joystick.back().onTrue(new InstantCommand(() -> drivetrain.visionGyroReset()));
  joystick.start().onTrue(drivetrain.resetGyro());
  joystick.a().toggleOnTrue(zoneCommand);

  // //passing

  // //climb
  // joystick.y().onTrue(climbGoesUpCommand);
  // joystick.a().onTrue(climbGoesDownCommand);

  // joystick.a().whileTrue(new InstantCommand(() -> shooterSubsystem.extendActuator(0.38)));
  // joystick.b().whileTrue(new InstantCommand(() -> shooterSubsystem.extendActuator(0.02)));
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
