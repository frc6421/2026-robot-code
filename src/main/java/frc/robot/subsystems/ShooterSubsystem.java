// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Methods: setRPM
 * gear ratio
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  final private TalonFX shooterMotor;
  
  final private TalonFXConfiguration shooterMotorConfig;

  public static final class ShooterConstants{
    private static final int  SHOOTER_CAN_ID = 0;

    private static final double SHOOTER_GEAR_RATIO = 0.0;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG = new MotorOutputConfigs();

  }
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.SHOOTER_CAN_ID);

    shooterMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
