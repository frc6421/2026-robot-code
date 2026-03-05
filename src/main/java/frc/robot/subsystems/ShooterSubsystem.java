// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Methods: setRPM
 * gear ratio
 */

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants;
import frc.robot.SOTMTable;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  final private TalonFX shooterMotorLeft;
  final private TalonFX shooterMotorRight;
  final private TalonFX shooterMotorTurn;

  private final Servo leftActuator;
  private final Servo rightActuator;
  
  final private TalonFXConfiguration shooterMotorConfig;
  final private TalonFXConfiguration shooterMotorTurnConfig;

  private Slot0Configs PIDConfigShooter;
  private Slot0Configs PIDConfigTurn;

  private VelocityVoltage shooterRequest;
  private PositionVoltage shooterRequestTurn;

  private double setAngle = 0.0;
  private double shotAngle = 0.0;
  private double horizSpeed = 0.0;

  private Mechanism2d turretmech = new Mechanism2d(2, 2);
  private MechanismRoot2d turretRoot = turretmech.getRoot("TurretRoot", 1, 1);
  private MechanismLigament2d turretLigament = turretRoot.append(new MechanismLigament2d("TurretLigament", 0.5, 180));
  private TalonFXSimState shooterTurnSim;
  private SingleJointedArmSim shooterTurnPhysics;

  public static final class ShooterConstants{
    private static final int SHOOTER_LEFT_CAN_ID = 51;
    private static final int SHOOTER_RIGHT_CAN_ID = 52;
    private static final int SHOOTER_STEER_CAN_ID = 50;

    private static final double SHOOTER_GEAR_RATIO = 0.0;
    private static final double SHOOTER_HOOD_MAXIMUM_ANGLE = 65.0; //This is when the hood is down as far as possible
    private static final double SHOOTER_HOOD_MAX_CHANGE_ANGLE = 25.0; //How much the hood can actuate from bottom to top in degrees

    private static final double SHOOTER_ROTATIONS_PER_DEGREE = 0.13568;
    private static final double MAX_RPM_ERROR = 50.0;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);
    private static final MotorOutputConfigs SHOOTER_MOTOR_TURN_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake)
    .withInverted(InvertedValue.Clockwise_Positive);

    private static final SoftwareLimitSwitchConfigs SHOOTER_TURN_SOFT_LIMITS = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(true)
    .withReverseSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(14.7)
    .withReverseSoftLimitThreshold(-15.0);

    private static final class SHOOTER_PID_VALUES {
    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.065;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
    }

    private static final class SHOOTER_TURN_PID_VALUES {
    private static final double kP = 2.015;

    private static final double kS = 0.3835;

    private static final double kV = 0.1;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
    }

  }
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_LEFT_CAN_ID);
    shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_RIGHT_CAN_ID);
    shooterMotorTurn = new TalonFX(ShooterConstants.SHOOTER_STEER_CAN_ID);

    leftActuator = new Servo(0);
    rightActuator = new Servo(1);

    leftActuator.enableDeadbandElimination(true);
    rightActuator.enableDeadbandElimination(true);

    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);

    PIDConfigShooter = new Slot0Configs();

    PIDConfigShooter.kS = ShooterConstants.SHOOTER_PID_VALUES.kS;
    PIDConfigShooter.kP = ShooterConstants.SHOOTER_PID_VALUES.kP;
    PIDConfigShooter.kV = ShooterConstants.SHOOTER_PID_VALUES.kV;
    PIDConfigShooter.kI = ShooterConstants.SHOOTER_PID_VALUES.kI;
    PIDConfigShooter.kD = ShooterConstants.SHOOTER_PID_VALUES.kD;

    PIDConfigTurn = new Slot0Configs();

    PIDConfigTurn.kS = ShooterConstants.SHOOTER_TURN_PID_VALUES.kS;
    PIDConfigTurn.kP = ShooterConstants.SHOOTER_TURN_PID_VALUES.kP;
    PIDConfigTurn.kV = ShooterConstants.SHOOTER_TURN_PID_VALUES.kV;
    PIDConfigTurn.kI = ShooterConstants.SHOOTER_TURN_PID_VALUES.kI;
    PIDConfigTurn.kD = ShooterConstants.SHOOTER_TURN_PID_VALUES.kD;

    shooterMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG);

    shooterMotorTurnConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_TURN_CONFIG)
    .withSoftwareLimitSwitch(ShooterConstants.SHOOTER_TURN_SOFT_LIMITS)
    .withSlot0(PIDConfigTurn);

    shooterRequest = new VelocityVoltage(0).withEnableFOC(true);

    shooterRequestTurn = new PositionVoltage(0)
    .withEnableFOC(true)
    .withSlot(0);
    
    shooterMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorTurn.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotorLeft.getConfigurator().apply(shooterMotorConfig);
    shooterMotorRight.getConfigurator().apply(shooterMotorConfig);
    shooterMotorTurn.getConfigurator().apply(shooterMotorTurnConfig);

    shooterMotorLeft.getConfigurator().apply(PIDConfigShooter);
    shooterMotorRight.getConfigurator().apply(PIDConfigShooter);
    shooterMotorTurn.getConfigurator().apply(PIDConfigTurn);

    shooterMotorRight.setControl(new Follower(ShooterConstants.SHOOTER_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
    shooterMotorTurn.setPosition(0);
    SmartDashboard.putData("Shooter", this);
    SmartDashboard.putData("TurretMech", turretmech);

    shooterTurnSim = shooterMotorTurn.getSimState();

    shooterTurnPhysics = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
     1/ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE,
      0.05,
      0.3,
      Math.toRadians(-90),
      Math.toRadians(90),
      false,
      0);

  }
  
  public void setRPM(double rpm) {

    double rps = rpm/60.0;

    shooterRequest = shooterRequest.withVelocity(rps).withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);
    shooterMotorLeft.setControl(shooterRequest);
  }

  public Command setRPMAuto(double rpm) {
return runOnce(() -> {
    double rps = rpm/60.0;

    shooterRequest = shooterRequest.withVelocity(rps).withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);
    shooterMotorLeft.setControl(shooterRequest);
});
  }

  public double getRPM() {
    return (shooterMotorLeft.getVelocity().getValueAsDouble() * 60);
  }

  public boolean withinErrorRPM(double setRPM) {
    return (Math.abs(getRPM() - setRPM) <= ShooterConstants.MAX_RPM_ERROR);
  }

  public void turnShooter(double angle) {
      shooterMotorTurn.setControl(shooterRequestTurn.withPosition(angle * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE));
  }

  public void setShooterTurnVoltage(double voltage) {
    shooterMotorTurn.setVoltage(voltage);
  }

  public void extendActuator(double length) {
    leftActuator.set(length);
    rightActuator.set(length);
  }

  public void setHoodAngle(double angle) {
    double setAngle = angle;
    if ((angle > ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE)) {
      setAngle = ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE;
    }
    else if (angle < (ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE - ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE)) {
      setAngle = ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE - ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE;
    }
    leftActuator.set(Math.abs(getHoodAngle() - setAngle) / ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE);
    rightActuator.set(Math.abs(getHoodAngle() - setAngle) / ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE);
  }

  public double getActuatorLength() {
    return leftActuator.get();
  }

  /**
   * Uses the LinearActuators to find the angle of the hood
   * @return hood angle (degrees)
   */
  public double getHoodAngle() {
    return ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE - (getActuatorLength() * ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE);
  }

  public double getShooterAngle() {
    return shooterMotorTurn.getRotorPosition().getValueAsDouble() / ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE;
  }
  public void setOutput(double output) {
    shooterMotorLeft.set(output);
  }

  public void stopShooter(){
    shooterMotorLeft.stopMotor();
  }

  public void stopShooterTurn() {
    shooterMotorTurn.stopMotor();
  }

  public void updateSOTM(Pose2d robotPose, ChassisSpeeds robotSpeed) {

        // // 1. LATENCY COMP
        // double latency = 0.15; // Tuned constant
        // Translation2d futurePos = robotPose.getTranslation().plus(
        //     new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        // );

        // // 2. GET TARGET VECTOR
        // Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_HUB_GOAL : TrajectoryConstants.RED_HUB_GOAL;
        // Translation2d targetVec = goalLocation.minus(futurePos);
        // double dist = targetVec.getNorm();

        // // 3. CALCULATE IDEAL SHOT (Stationary)
        // // Note: This returns HORIZONTAL velocity component
        // double idealHorizontalSpeed = (SOTMTable.getSpeed(dist) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
        // double idealVerticalSpeed = (SOTMTable.getSpeed(dist) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

        // // 4. VECTOR SUBTRACTION
        // Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        // Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // // 5. CONVERT TO CONTROLS
        // double turretAngle = shotVec.getAngle().getDegrees();
        // double newHorizontalSpeed = shotVec.getNorm();
        

        // // 6. SOLVE FOR NEW PITCH/RPM
        // // Assuming constant total exit velocity, variable hood:
        // double totalExitVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed); // m/s
        // // Clamp to avoid domain errors if we need more speed than possible
        // double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        // double newPitch = Math.acos(ratio);

        // 7. SET OUTPUTS
        // this.turnShooter(MathUtil.inputModulus(turretAngle - 180, -180, 180));
        // this.setRPM(totalExitVelocity / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * 60);
        // //Latency
        double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency).rotateBy(robotPose.getRotation())
        );

        //get Target Vecotr
        Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_HUB_GOAL : TrajectoryConstants.RED_HUB_GOAL;
        Translation2d targetVector = goalLocation.minus(futurePos);
        double distance = targetVector.getNorm();

        double idealHorizontalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
        double idealVerticalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

        //minus vectors
        Translation2d robotVelocityVector = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        //get the angles and stuff
        double turretAngle = shotVector.getAngle().minus(robotPose.getRotation()).getDegrees();
        double newHorizontalSpeed = shotVector.getNorm();

        this.setRPM((Math.hypot(newHorizontalSpeed, idealVerticalSpeed) / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
        this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));
        this.setHoodAngle(Constants.ShooterConstants.HOOD_ANGLE_SHOOT);

        // setAngle = MathUtil.inputModulus(turretAngle - 180.0, -180, 180);
        turretLigament.setAngle(MathUtil.inputModulus(360 - turretAngle - 180, -180, 180) + 180);
        //shotAngle = shotVector.getAngle().getDegrees();
        horizSpeed = idealHorizontalSpeed;
        
    }

  public void updateHighPOTM(Pose2d robotPose, ChassisSpeeds robotSpeed) {

        double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency).rotateBy(robotPose.getRotation())
        );

        //get Target Vecotr
        Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_BUMP_D_L.getTranslation() : TrajectoryConstants.RED_BUMP_O_L.getTranslation();
        Translation2d targetVector = goalLocation.minus(futurePos);
        double distance = targetVector.getNorm();

        double idealHorizontalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
        double idealVerticalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

        //minus vectors
        Translation2d robotVelocityVector = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        //get the angles and stuff
        double turretAngle = shotVector.getAngle().minus(robotPose.getRotation()).getDegrees();
        double newHorizontalSpeed = shotVector.getNorm();
         // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed); // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        this.setRPM((Math.hypot(newHorizontalSpeed, idealVerticalSpeed) / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
        this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));
        this.setHoodAngle(newPitch);

        // setAngle = MathUtil.inputModulus(turretAngle - 180.0, -180, 180);
        turretLigament.setAngle(MathUtil.inputModulus(360 - turretAngle - 180, -180, 180) + 180);
        //shotAngle = shotVector.getAngle().getDegrees();
        horizSpeed = idealHorizontalSpeed; 
    }

  public void updateLowPOTM(Pose2d robotPose, ChassisSpeeds robotSpeed) {

        double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency).rotateBy(robotPose.getRotation())
        );

        //get Target Vecotr
        Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_BUMP_O_L.getTranslation() : TrajectoryConstants.RED_BUMP_D_L.getTranslation();
        Translation2d targetVector = goalLocation.minus(futurePos);
        double distance = targetVector.getNorm();

        double idealHorizontalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
        double idealVerticalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

        //minus vectors
        Translation2d robotVelocityVector = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        //get the angles and stuff
        double turretAngle = shotVector.getAngle().minus(robotPose.getRotation()).getDegrees();
        double newHorizontalSpeed = shotVector.getNorm();
         // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed); // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        this.setRPM((Math.hypot(newHorizontalSpeed, idealVerticalSpeed) / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
        this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));
        this.setHoodAngle(newPitch);

        // setAngle = MathUtil.inputModulus(turretAngle - 180.0, -180, 180);
        turretLigament.setAngle(MathUtil.inputModulus(360 - turretAngle - 180, -180, 180) + 180);
        //shotAngle = shotVector.getAngle().getDegrees();
        horizSpeed = idealHorizontalSpeed; 
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);

    builder.addDoubleProperty("Shooter RPM", () -> shooterMotorLeft.getVelocity().getValueAsDouble()*60, null);
    builder.addDoubleProperty("Shooter Voltage", () -> shooterMotorLeft.getMotorVoltage().getValueAsDouble(), null);
    

    builder.addDoubleProperty("ShooterTurn SetAngle", () -> setAngle, null);
    builder.addDoubleProperty("Shooter Angle", () -> getShooterAngle(), null);
    builder.addDoubleProperty("TargetHorizontalSpeed", () -> horizSpeed, null);

    builder.addDoubleProperty("Actuator Length", () -> leftActuator.getPosition(), null);
  }

public void simulationPeriodic() {
  shooterTurnPhysics.setInput(shooterTurnSim.getMotorVoltage());
  shooterTurnPhysics.update(.020);

   shooterTurnSim.setRawRotorPosition(Math.toDegrees(shooterTurnPhysics.getAngleRads()) * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
   shooterTurnSim.setRotorVelocity(Math.toDegrees(shooterTurnPhysics.getAngleRads()) * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
}

}

