// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Sensor
 * Methods: climbUp, climbDown
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase {
  public TalonFX climbMotor;
  private TalonFXConfiguration climbMotorConfig;
  public MotionMagicVoltage climbRequest;
  public PositionVoltage voltageRequest;
  /** Creates a new ClimbSubsystem. */
  public static class ClimbConstants { 

  private static final int CLIMB_MOTOR_ID = 31;

  private static final double MAX_ERROR_INCHES = 0.005;

  public static final double CLIMBER_INCHES_PER_ROTATION = 0.026415; //TODO find real value

  // private static final double MAX_VELOCITY = 50.0; //rotations per second

  // private static final double MAX_ACCEL = 250.0; //rotations per second^2

  // private static final double MAX_JERK = 2500.0; //rotations per second^3

  // private static final MotionMagicConfigs CLIMB_MOTION_CONFIG = new MotionMagicConfigs()
  // .withMotionMagicCruiseVelocity(MAX_VELOCITY)
  // .withMotionMagicAcceleration(MAX_ACCEL)
  // .withMotionMagicJerk(MAX_JERK);

  private static final Voltage MAX_VOLTS = Volts.of(32);

  private static final VoltageConfigs CLIMBER_VOLTAGE_CONFIGS = new VoltageConfigs()
  .withPeakForwardVoltage(MAX_VOLTS)
  .withPeakReverseVoltage(MAX_VOLTS.times(-1.0));
    
  private static final MotorOutputConfigs CLIMB_MOTOR_CONFIGS = new MotorOutputConfigs()
  .withNeutralMode(NeutralModeValue.Brake)
  .withInverted(InvertedValue.Clockwise_Positive);

  private static final Current CLIMB_CURRENT_LIMIT = Amps.of(250);

  private static final CurrentLimitsConfigs CLIMB_CURRENT_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);
  

  private static final double FORWARD_SOFT_LIMIT = 265.0;
  private static final double REVERSE_SOFT_LIMIT = 0.0;

  private static final SoftwareLimitSwitchConfigs CLIMB_SOFTWARE_CONFIGS = new SoftwareLimitSwitchConfigs()
  .withForwardSoftLimitEnable(true)
  .withReverseSoftLimitEnable(true)
  .withForwardSoftLimitThreshold(ClimbConstants.FORWARD_SOFT_LIMIT)
  .withReverseSoftLimitThreshold(ClimbConstants.REVERSE_SOFT_LIMIT);

  private static final double kP = 35.6;
  private static final double kS = -.19;
  private static final double kV = .423;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  
  private static final Slot0Configs CLIMBER_PID_CONFIGS = new Slot0Configs()
    .withKS(kS)
    .withKP(kP)
    .withKV(kV)
    .withKI(kI)
    .withKD(kD);
    } 

  public ClimbSubsystem() {
    
    climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
    climbRequest = new MotionMagicVoltage(0);
    voltageRequest = new PositionVoltage(0);

    RobotContainer.applyTalonConfigs(climbMotor, new TalonFXConfiguration());
    
    climbMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ClimbConstants.CLIMB_MOTOR_CONFIGS)
    .withCurrentLimits(ClimbConstants.CLIMB_CURRENT_CONFIGS)
    //.withMotionMagic(ClimbConstants.CLIMB_MOTION_CONFIG)
    .withVoltage(ClimbConstants.CLIMBER_VOLTAGE_CONFIGS)
    .withSlot0(ClimbConstants.CLIMBER_PID_CONFIGS)
    .withSoftwareLimitSwitch(ClimbConstants.CLIMB_SOFTWARE_CONFIGS);

    RobotContainer.applyTalonConfigs(climbMotor, climbMotorConfig);
    SmartDashboard.putData("Climber" , this);

    climbMotor.setPosition(0);
  }

  public void setSpeed(double speed){
    climbMotor.set(speed);
  }

  public void setVoltage(double voltage){
    climbMotor.setVoltage(voltage);
  }

  public void setVelocity(double velocity){
    climbMotor.setControl(new VelocityVoltage(velocity));
  }

  public boolean withinError(double position){
    return(Math.abs(getClimberHeight() - position) <= ClimbConstants.MAX_ERROR_INCHES);
  }

  private boolean withinError(DoubleSupplier position){
    return(Math.abs(getClimberHeight() - position.getAsDouble()) <= ClimbConstants.MAX_ERROR_INCHES);
  }
  /**
   * returns in inches
   */
  public double getClimberHeight(){
    return(climbMotor.getPosition().getValueAsDouble() * ClimbConstants.CLIMBER_INCHES_PER_ROTATION);
  }

  public void stopClimbMotors(){
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Motor Rotations", () -> getClimberHeight() / ClimbConstants.CLIMBER_INCHES_PER_ROTATION, null);
    builder.addDoubleProperty("Climber Velocity", () -> climbMotor.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("Climber Voltage", () -> climbMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("Climber Current", () -> climbMotor.getStatorCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Climber Height", () -> getClimberHeight(), null);
    builder.addBooleanProperty("Climber Within Error", () -> withinError(Constants.ClimbPositions.L1_INCHES), null);
  }
}
