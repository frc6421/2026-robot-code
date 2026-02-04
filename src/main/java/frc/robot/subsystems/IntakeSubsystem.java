// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Methods: setIntakeSpeed, stopIntake
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX intakePivot;
  private TalonFXConfiguration intakeMotorConfig;
  private PositionVoltage intakeRequestTurn;

  private Slot0Configs PIDConfigIntake;

  public static class IntakeConstants{

    private static final int INTAKE_MOTONER_ID = 60; 
    private static final int INTAKE_PIVOT_ID = 61;
   
    private static final MotorOutputConfigs INTAKE_MOTOR_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);

    private static final double INTAKE_ROTATIONS_PER_DEGREE = 0.0;

    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.1;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTONER_ID);
  intakePivot = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);
  
  RobotContainer.applyTalonConfigs(intakeMotor, new TalonFXConfiguration());
  RobotContainer.applyTalonConfigs(intakePivot, new TalonFXConfiguration());

  PIDConfigIntake = new Slot0Configs();
  PIDConfigIntake.kS = IntakeConstants.kS;
  PIDConfigIntake.kP = IntakeConstants.kP;
  PIDConfigIntake.kV = IntakeConstants.kV;
  PIDConfigIntake.kI = IntakeConstants.kI;
  PIDConfigIntake.kD = IntakeConstants.kD;

  intakeMotorConfig = new TalonFXConfiguration()
  .withMotorOutput(IntakeConstants.INTAKE_MOTOR_CONFIGS);

  RobotContainer.applyTalonConfigs(intakeMotor, intakeMotorConfig);
  RobotContainer.applyTalonConfigs(intakePivot, intakeMotorConfig);

  intakePivot.getConfigurator().apply(PIDConfigIntake);

  intakeRequestTurn = new PositionVoltage(0).withEnableFOC(true);
  }

  private void turnIntake(double angle) {

    intakeRequestTurn.withPosition(angle * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);

    intakePivot.setControl(intakeRequestTurn);

    
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
