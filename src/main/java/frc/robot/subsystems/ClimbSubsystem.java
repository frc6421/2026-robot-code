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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX climbMotor;
  private TalonFXConfiguration climbMotorConfig;
  /** Creates a new ClimbSubsystem. */


  public static class ClimbConstants { //lacking current limit

  private static final int CLIMB_MOTOR_ID = 30;
    
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

  public void setPosition(TalonFX motor, double position){
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i< 5; i++){
      status = motor.setPosition(position);
      if(status.isOK()){
        break;
      }
    }
    if (!status.isOK()){
      DataLogManager.log("Set Position Error" + motor.getDescription() + "Status code: " + status.toString());
    }
  }

  public void stopClimbMotors(){
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
