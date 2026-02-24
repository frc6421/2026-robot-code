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

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

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

  private DoubleSupplier setAngle = () -> 0.0;

  private TalonFXSimState shooterTurnSim;

  private SingleJointedArmSim shooterTurnPhysics;



  public static final class ShooterConstants{
    private static final int SHOOTER_LEFT_CAN_ID = 51;
    private static final int SHOOTER_RIGHT_CAN_ID = 52;
    private static final int SHOOTER_STEER_CAN_ID = 50;

    private static final double SHOOTER_GEAR_RATIO = 0.0;

    private static final double SHOOTER_ROTATIONS_PER_DEGREE = 0.1;
    private static final double MAX_RPM_ERROR = 50.0;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);
    private static final MotorOutputConfigs SHOOTER_MOTOR_TURN_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake)
    .withInverted(InvertedValue.Clockwise_Positive);

    private static final class SHOOTER_PID_VALUES {
    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.065;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
    }

    private static final class SHOOTER_TURN_PID_VALUES {
    private static final double kP = 0.1;

    private static final double kS = 0.1;

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

    shooterMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG);

    shooterMotorTurnConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_TURN_CONFIG);

    shooterRequest = new VelocityVoltage(0).withEnableFOC(true);

    shooterRequestTurn = new PositionVoltage(0).withEnableFOC(true);
    
    shooterMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorTurn.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotorLeft.getConfigurator().apply(shooterMotorConfig);
    shooterMotorRight.getConfigurator().apply(shooterMotorConfig);
    shooterMotorTurn.getConfigurator().apply(shooterMotorTurnConfig);

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

    shooterMotorLeft.getConfigurator().apply(PIDConfigShooter);
    shooterMotorRight.getConfigurator().apply(PIDConfigShooter);
    shooterMotorTurn.getConfigurator().apply(PIDConfigTurn);

    shooterMotorRight.setControl(new Follower(ShooterConstants.SHOOTER_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
    shooterMotorTurn.setPosition(0);
    SmartDashboard.putData("Shooter", this);

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

    shooterRequest.withVelocity(rps).withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);

    shooterMotorLeft.setControl(shooterRequest);
  }

  public double getRPM() {
    return (shooterMotorLeft.getVelocity().getValueAsDouble() * 60);
  }

  public boolean withinErrorRPM(double setRPM) {
    return (Math.abs(getRPM() - setRPM) <= ShooterConstants.MAX_RPM_ERROR);
  }

  private void turnShooter(double angle) {
    setAngle = () -> angle;
    shooterRequestTurn.withPosition(angle * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);

    shooterMotorTurn.setControl(shooterRequestTurn);
  }

  public void extendActuator(double length) {
    leftActuator.set(length);
    rightActuator.set(length);
  }

  public void setOutput(double output) {
    shooterMotorLeft.set(output);
  }

  public void stopShooter(){
    shooterMotorLeft.stopMotor();
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
    //C'est Turner.
    builder.addDoubleProperty("ShooterTurn Voltage", () -> shooterMotorTurn.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Velocity", () -> shooterMotorTurn.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Rotations", () -> shooterMotorTurn.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Angle", () -> shooterMotorTurn.getPosition().getValueAsDouble() / ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE, null);
    builder.addDoubleProperty("ShooterTurn SetAngle", setAngle, null);

    builder.addDoubleProperty("Actuator Length", () -> leftActuator.getPosition(), null);
  }

public void simulationPeriodic() {
  shooterTurnPhysics.setInput(shooterTurnSim.getMotorVoltage());
  shooterTurnPhysics.update(.020);

   shooterTurnSim.setRawRotorPosition(Math.toDegrees(shooterTurnPhysics.getAngleRads()) * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
   shooterTurnSim.setRotorVelocity(Math.toDegrees(shooterTurnPhysics.getAngleRads()) * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
}

}

