// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Methods: setIntakeSpeed, stopIntake
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX intakeMotwo;
  private TalonFXConfiguration intakeMotorConfig;

  public static class IntakeConstants{

    private static final int INTAKE_MOTONER_ID = 60; 
    private static final int INTAKE_MOTWOR_ID = 61;
   
    private static final MotorOutputConfigs INTAKE_MOTOR_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTONER_ID);
  intakeMotwo = new TalonFX(IntakeConstants.INTAKE_MOTWOR_ID);
  
  RobotContainer.applyTalonConfigs(intakeMotor, new TalonFXConfiguration());
  RobotContainer.applyTalonConfigs(intakeMotwo, new TalonFXConfiguration());

  intakeMotorConfig = new TalonFXConfiguration()
  .withMotorOutput(IntakeConstants.INTAKE_MOTOR_CONFIGS);

  RobotContainer.applyTalonConfigs(intakeMotor, intakeMotorConfig);
  RobotContainer.applyTalonConfigs(intakeMotwo, intakeMotorConfig);
  }

  public Command setIntakeSpeed(double output){
    return runOnce(() -> intakeMotor.set(output));
  }

  public Command stopIntake(){
    return runOnce(() -> intakeMotor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
