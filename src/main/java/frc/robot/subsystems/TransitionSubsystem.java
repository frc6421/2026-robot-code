// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

public class TransitionSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
private TalonFX shooterTransition;
private TalonFX hopperTransition;
private TalonFXConfiguration transitionConfig;

public static class TransitionConstants{

private static final int SHOOTER_TRANSITION_ID = 20;
private static final int HOPPER_TRANSITION_ID = 21;

private static final MotorOutputConfigs TRANSITION_MOTOR_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive);
}

 public TransitionSubsystem() {
  shooterTransition = new TalonFX(TransitionConstants.SHOOTER_TRANSITION_ID);
  hopperTransition = new TalonFX(TransitionConstants.HOPPER_TRANSITION_ID);
  
  RobotContainer.applyTalonConfigs(shooterTransition, new TalonFXConfiguration());
  RobotContainer.applyTalonConfigs(hopperTransition, new TalonFXConfiguration());

  transitionConfig = new TalonFXConfiguration()
  .withMotorOutput(TransitionConstants.TRANSITION_MOTOR_CONFIGS);

  RobotContainer.applyTalonConfigs(shooterTransition, transitionConfig);
  RobotContainer.applyTalonConfigs(hopperTransition, transitionConfig);

  SmartDashboard.putData("Transition", this);
  }

public void intakeTransition(double output){
      hopperTransition.set(output);
  }

public void shooterTransition(double shooterOutput, double transitionOutput){
      shooterTransition.set(shooterOutput);
      hopperTransition.set(transitionOutput);
  }

public void passingTransition(double shooterOutput, double transitionOutput){
      shooterTransition.set(shooterOutput);
      hopperTransition.set(transitionOutput);
  }

public void stopTransition(){
  shooterTransition.stopMotor();
  hopperTransition.stopMotor();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);

      builder.addDoubleProperty("ShooterTransitionRPM", () -> shooterTransition.getVelocity().getValueAsDouble() * 60, null);
  }
}
