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
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  final private TalonFX shooterMotor;
  final private TalonFX shooterMotorTurn;
  
  final private TalonFXConfiguration shooterMotorConfig;
  final private TalonFXConfiguration shooterMotorTurnConfig;

  private Slot0Configs PIDConfigShooter;
  private Slot0Configs PIDConfigTurn;

  private VelocityVoltage shooterRequest;
  private PositionVoltage shooterRequestTurn;

  private DoubleSupplier setAngle = () -> 0.0;

  public static final class ShooterConstants{
    private static final int SHOOTER_LEFT_CAN_ID = 0;
    private static final int SHOOTER_RIGHT_CAN_ID = 0;

    private static final double SHOOTER_GEAR_RATIO = 0.0;

    private static final double SHOOTER_ROTATIONS_PER_DEGREE = 0.1;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast);
    private static final MotorOutputConfigs SHOOTER_MOTOR_TURN_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake);

    private static final class SHOOTER_PID_VALUES {
    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.1;

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
    shooterMotor = new TalonFX(ShooterConstants.SHOOTER_LEFT_CAN_ID);
    shooterMotorTurn = new TalonFX(ShooterConstants.SHOOTER_RIGHT_CAN_ID);

    shooterMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG);

    shooterMotorTurnConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_TURN_CONFIG);

    shooterRequest = new VelocityVoltage(0).withEnableFOC(true);

    shooterRequestTurn = new PositionVoltage(0).withEnableFOC(true);
    
    shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorTurn.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
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

    shooterMotor.getConfigurator().apply(PIDConfigShooter);
    shooterMotorTurn.getConfigurator().apply(PIDConfigTurn);
    SmartDashboard.putData("Shooter", this);
  }
  
  private void setRPM(double rpm) {

    double rps = rpm/60.0;

    shooterRequest.withVelocity(rps).withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);

    shooterMotor.setControl(shooterRequest);
  }

  private void turnShooter(double angle) {
    setAngle = () -> angle;
    shooterRequestTurn.withPosition(angle * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);

    shooterMotorTurn.setControl(shooterRequestTurn);

    
  }

  private void stop(){
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);

    builder.addDoubleProperty("Shooter RPM", () -> shooterMotor.getVelocity().getValueAsDouble()*60, null);
    builder.addDoubleProperty("Shooter Voltage", () -> shooterMotor.getMotorVoltage().getValueAsDouble(), null);
    //C'est Turner.
    builder.addDoubleProperty("ShooterTurn Voltage", () -> shooterMotorTurn.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Velocity", () -> shooterMotorTurn.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Rotations", () -> shooterMotorTurn.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("ShooterTurn Angle", () -> shooterMotorTurn.getPosition().getValueAsDouble() / ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE, null);
    builder.addDoubleProperty("ShooterTurn SetAngle", setAngle, null);
  }
}
