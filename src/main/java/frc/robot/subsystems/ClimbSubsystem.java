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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX climbMotor;
  private TalonFXConfiguration climbMotorConfig;
  private MotionMagicVoltage climbRequest;
  private PositionVoltage voltageRequest;
  /** Creates a new ClimbSubsystem. */


  public static class ClimbConstants { 

  private static final int CLIMB_MOTOR_ID = 30;

  private static final double MAX_ERROR_INCHES = 1.0;

  private static final double CLIMBER_INCHES_PER_ROTATION = 3.0; //TODO find real value

  private static final double MAX_VELOCITY = 15.0; //rotations per second

  private static final double MAX_ACCEL = 100.0; //rotations per second^2

  private static final double MAX_JERK = 3000.0; //rotations per second^3

  private static final MotionMagicConfigs CLIMB_MOTION_CONFIG = new MotionMagicConfigs()
  .withMotionMagicCruiseVelocity(MAX_VELOCITY)
  .withMotionMagicAcceleration(MAX_ACCEL)
  .withMotionMagicJerk(MAX_JERK);

  private static final Voltage MAX_VOLTS = Volts.of(11);

  private static final VoltageConfigs CLIMBER_VOLTAGE_CONFIGS = new VoltageConfigs()
  .withPeakForwardVoltage(MAX_VOLTS)
  .withPeakReverseVoltage(MAX_VOLTS);
    
  private static final MotorOutputConfigs CLIMB_MOTOR_CONFIGS = new MotorOutputConfigs()
  .withNeutralMode(NeutralModeValue.Coast)
  .withInverted(InvertedValue.Clockwise_Positive);

  private static final Current CLIMB_CURRENT_LIMIT = Amps.of(250);

   private static final CurrentLimitsConfigs CLIMB_CURRENT_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true);

  } 

  public ClimbSubsystem() {
    
    climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);

    RobotContainer.applyTalonConfigs(climbMotor, new TalonFXConfiguration());
    
    climbMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ClimbConstants.CLIMB_MOTOR_CONFIGS)
    .withCurrentLimits(ClimbConstants.CLIMB_CURRENT_CONFIGS);
    RobotContainer.applyTalonConfigs(climbMotor, climbMotorConfig);
  }

  public Command setPosition(double position){
    return
    run(()->{
      climbMotor.setControl(climbRequest.withPosition(position / ClimbConstants.CLIMBER_INCHES_PER_ROTATION));
    })
    .until(()->withinError(position));
  }

  private boolean withinError(double position){
    return(Math.abs(getClimberHeight() - position) < ClimbConstants.MAX_ERROR_INCHES);
  }
  /**
   * returns in inches
   */
  private double getClimberHeight(){
    return(climbMotor.getPosition().getValueAsDouble() * ClimbConstants.CLIMBER_INCHES_PER_ROTATION);
  }

  public void stopClimbMotors(){
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
