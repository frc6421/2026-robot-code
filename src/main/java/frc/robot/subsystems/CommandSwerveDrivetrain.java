package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.WarriorCamera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {    

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentricFacingAngle alignAngleRequest = new FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final SwerveRequest.RobotCentric nudgeRequest = new RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity);

    public final WarriorCamera backLeftCamera = new WarriorCamera("Camera_6_OV9281_USB_Camera", WarriorCamera.CameraConstants.FRONT_LEFT_TRANSFORM3D);
    public final WarriorCamera backRightCamera = new WarriorCamera("Camera_2_OV9281_USB_Camera", WarriorCamera.CameraConstants.FRONT_RIGHT_TRANSFORM3D);
    public final PhotonCamera allignCamera = new PhotonCamera("Camera_4_OV9281_USB_Camera");

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
      new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
          AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));

    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
      // Position controllers
      new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
      new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
      thetaController);

    //Align PID Controllers
    private final PIDController xController = new PIDController(AlignConstants.ALIGN_P,
        AutoConstants.X_DRIVE_I,
        AutoConstants.X_DRIVE_D);
    private final PIDController yController = new PIDController(AlignConstants.ALIGN_P,
        AutoConstants.Y_DRIVE_I,
        AutoConstants.Y_DRIVE_D);
    
    private final ApplyRobotSpeeds autoApplyRobotSpeeds = new ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Reset Gyro", resetGyro());
        SmartDashboard.putData("xController", xController);
        SmartDashboard.putData("yController", yController);
        SmartDashboard.putData("Heading Controller", alignAngleRequest.HeadingController);
        allignCamera.setDriverMode(true);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public void updatePose(WarriorCamera camera) {
        if(camera.filterOdometry()) {
                addVisionMeasurement(
                new Pose2d(camera.getPose2d().getX(),
                camera.getPose2d().getY(),
                getPigeon2().getRotation2d()),
                Utils.fpgaToCurrentTime(camera.getTimer()),
                camera.getStandardDeviation());
            }
    }

    public Command reefAlignCommand(Supplier<Pose2d> targetPose) {
        alignAngleRequest.HeadingController.setP(AutoConstants.THETA_P);
        alignAngleRequest.HeadingController.setTolerance(Units.degreesToRadians(0.5), Units.degreesToRadians(0.5));
        xController.setTolerance(.015, .1);
        yController.setTolerance(.015, .1);
        return applyRequest(() ->  { 
          Pose2d currentPose = getState().Pose;
          double xVelocity = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.get().getX()), -2.5, 2.5);
          double yVelocity = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.get().getY()), -2.5, 2.5);

          return alignAngleRequest.withTargetDirection(targetPose.get().getRotation()).withVelocityX(xVelocity).withVelocityY(yVelocity);
        }).until(() -> xController.atSetpoint() && yController.atSetpoint() && alignAngleRequest.HeadingController.atSetpoint());
        //.andThen(this.runOnce(() -> alignAngleRequest.withVelocityX(0).withVelocityY(0)));
    }

    public Command sourceAlignCommand(Supplier<Pose2d> targetPose) {
        alignAngleRequest.HeadingController.setP(AutoConstants.THETA_P);
        alignAngleRequest.HeadingController.setTolerance(Units.degreesToRadians(1.0), Units.degreesToRadians(2.0));
        xController.setTolerance(.1, .1);
        yController.setTolerance(.1, .1);
        return applyRequest(() ->  { 
          Pose2d currentPose = getState().Pose;
          double xVelocity = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.get().getX()), -3.7, 3.7);
          double yVelocity = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.get().getY()), -3.7, 3.7);

          return alignAngleRequest.withTargetDirection(targetPose.get().getRotation()).withVelocityX(xVelocity).withVelocityY(yVelocity);
        }).until(() -> xController.atSetpoint() && yController.atSetpoint() && alignAngleRequest.HeadingController.atSetpoint())
        .andThen(this.runOnce(() -> alignAngleRequest.withVelocityX(0).withVelocityY(0)));
    }


    public Command offsetAlignCommand(Supplier<Pose2d> targetPose) {
        alignAngleRequest.HeadingController.setP(AutoConstants.THETA_P);
        alignAngleRequest.HeadingController.setTolerance(Units.degreesToRadians(5.0), Units.degreesToRadians(1.0));
        xController.setTolerance(.2, .1);
        yController.setTolerance(.2, .1);
        return applyRequest(() ->  { 
          Pose2d currentPose = getState().Pose;
          double xVelocity = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.get().getX()), -3.5, 3.5);
          double yVelocity = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.get().getY()), -3.5, 3.5);

          return alignAngleRequest.withTargetDirection(targetPose.get().getRotation()).withVelocityX(xVelocity).withVelocityY(yVelocity);
        }).until(() -> xController.atSetpoint() && yController.atSetpoint() && alignAngleRequest.HeadingController.atSetpoint())
        .andThen(this.runOnce(() -> alignAngleRequest.withVelocityX(0).withVelocityY(0)));
    }

    public Command nudgeCommand(double direction) {
        double velocity = 0.1;
        double time = 0.1;
        return this.applyRequest(() -> nudgeRequest.withVelocityY(velocity * direction))
        .withTimeout(time);
    }

    public Command nudgeForwardCommand() {
        double velocity = 0.2;
        double time = 0.5;
        return this.applyRequest(() -> nudgeRequest.withVelocityX(velocity))
        .withTimeout(time)
        .andThen(this.runOnce(() -> nudgeRequest.withVelocityX(0)));
    }

    public Command stopAlignCommand() {
        return this.runOnce(() -> alignAngleRequest.withVelocityX(0).withVelocityY(0));
    }

    public Command resetGyro() {
        return runOnce(() -> getPigeon2().reset());
    }

    public void visionGyroReset() {
        double cameraAngle = 0.0;


        if (backLeftCamera.hasTarget() && backRightCamera.hasTarget()) {
            if (backLeftCamera.getNumberOfTags() >= 2 && backRightCamera.getNumberOfTags() >= 2) {
                cameraAngle = 
            (backLeftCamera.getPose2d().getRotation().getDegrees() + 
            backRightCamera.getPose2d().getRotation().getDegrees()) / 2.0;
            }

            if (backLeftCamera.getNumberOfTags() < 2 && backRightCamera.getNumberOfTags() >= 2) {
                cameraAngle = backRightCamera.getPose2d().getRotation().getDegrees();
            }

            if (backLeftCamera.getNumberOfTags() >= 2 && backRightCamera.getNumberOfTags() < 2) {
                cameraAngle = backLeftCamera.getPose2d().getRotation().getDegrees();
            }

            if (backLeftCamera.getNumberOfTags() < 2 && backRightCamera.getNumberOfTags() < 2) {
                cameraAngle = 
            (backLeftCamera.getPose2d().getRotation().getDegrees() + 
            backRightCamera.getPose2d().getRotation().getDegrees()) / 2.0;
            }
        } 
        
        if (backLeftCamera.hasTarget() && !backRightCamera.hasTarget()) {
            cameraAngle = backLeftCamera.getPose2d().getRotation().getDegrees();
        }

        if (!backLeftCamera.hasTarget() && backRightCamera.hasTarget()) {
            cameraAngle = backRightCamera.getPose2d().getRotation().getDegrees();
        }

        if (!backLeftCamera.hasTarget() && !backRightCamera.hasTarget()) {
            cameraAngle = getPigeon2().getYaw().getValueAsDouble();
        }
        getPigeon2().setYaw(cameraAngle);
    }

    @Override
    public void periodic() {
        
            updatePose(backLeftCamera);
            updatePose(backRightCamera);
            //updatePose(backCamera);
        
        
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    public Command runTrajectoryCommand(Trajectory trajectory) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        final Timer timer = new Timer();
        return runOnce(timer::restart).andThen(applyRequest(() -> {
        double curTime = timer.get();
        var desiredState = trajectory.sample(curTime);
        ChassisSpeeds targetRobotSpeeds = holonomicDriveController.calculate(
            getState().Pose, desiredState,
            desiredState.poseMeters.getRotation());
        targetRobotSpeeds = ChassisSpeeds.discretize(targetRobotSpeeds, 0.020);
        return autoApplyRobotSpeeds.withSpeeds(targetRobotSpeeds);
        }))
            .until(() -> timer.hasElapsed(trajectory.getTotalTimeSeconds()));
  }

}