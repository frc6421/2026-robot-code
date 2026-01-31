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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  final private TalonFX shooterMotorLeft;
  final private TalonFX shooterMotorRight;
  
  final private TalonFXConfiguration shooterMotorConfig;

  private Slot0Configs PIDConfig;

  private VelocityVoltage shooterRequest;

  public static final class ShooterConstants{
    private static final int  SHOOTER_LEFT_CAN_ID = 0;
    private static final int  SHOOTER_RIGHT_CAN_ID = 0;

    private static final double SHOOTER_GEAR_RATIO = 0.0;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast);

    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.1;

    private static final double kI = 0.0;

    private static final double kD = 0.0;

    private static final double TARGET_RPM = 5000.0;

  }
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_LEFT_CAN_ID);
    shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_RIGHT_CAN_ID);

    shooterMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG);

    shooterRequest = new VelocityVoltage(0);
    
    shooterMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorRight.getConfigurator().apply(new TalonFXConfiguration());

    PIDConfig = new Slot0Configs();

    PIDConfig.kS = ShooterConstants.kS;
    PIDConfig.kP = ShooterConstants.kP;
    PIDConfig.kV = ShooterConstants.kV;
    PIDConfig.kI = ShooterConstants.kI;
    PIDConfig.kD = ShooterConstants.kD;

    shooterMotorLeft.getConfigurator().apply(PIDConfig);
    shooterMotorRight.getConfigurator().apply(PIDConfig);

    shooterMotorRight.setControl(new Follower(ShooterConstants.SHOOTER_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
  }
  
  private void setRPM(double rpm){

    double rps = rpm/60.0;

    shooterRequest.withVelocity(rps).withFeedForward(ShooterConstants.kV * rps);

    shooterMotorLeft.setControl(shooterRequest);
  }

  private void stop(){
    shooterMotorLeft.stopMotor();

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
