// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Setup motors
 * Methods: setIntakeSpeed, stopIntake
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakePositions;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX intakePivotRight;
  private TalonFX intakePivotLeft;
  private TalonFXConfiguration intakeMotorConfig;
  private TalonFXConfiguration intakePivotConfig;
  private PositionVoltage intakeRequestTurn;

  private Slot0Configs PIDConfigIntake;

  public static class IntakeConstants{
    //.6557rot 90 deg
    private static final int INTAKE_MOTOR_ID = 60; 
    private static final int INTAKE_PIVOT_RIGHT_ID = 62; 
    private static final int INTAKE_PIVOT_LEFT_ID = 61;
   
    private static final MotorOutputConfigs INTAKE_MOTOR_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final MotorOutputConfigs INTAKE_PIVOT_CONFIGS = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake)
    .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final double FORWARD_SOFT_LIMIT = 4.72;
    private static final double REVERSE_SOFT_LIMIT = 0.15;

    private static final double INTAKE_MOTOR_SPEED = IntakePositions.INTAKE_SPEED;
    private static final double INTAKE_PIVOT_SPEED = IntakePositions.INTAKE_PIVOT_SPEED;
    private static final double INTAKE_PIVOT_SPEED_IN = IntakePositions.INTAKE_PIVOT_SPEED_IN;

    private static final SoftwareLimitSwitchConfigs INTAKE_PIVOT_SOFTWARE_CONFIGS = new SoftwareLimitSwitchConfigs()
  .withForwardSoftLimitEnable(true)
  .withReverseSoftLimitEnable(true)
  .withForwardSoftLimitThreshold(IntakeConstants.FORWARD_SOFT_LIMIT)
  .withReverseSoftLimitThreshold(IntakeConstants.REVERSE_SOFT_LIMIT);

  private static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS = new CurrentLimitsConfigs()
  .withStatorCurrentLimit(80)
  .withStatorCurrentLimitEnable(true)
  .withSupplyCurrentLimit(80)
  .withSupplyCurrentLimitEnable(true);

    private static final double INTAKE_ROTATIONS_PER_DEGREE = 0.007285;

    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.1;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
  }

  // Simulation
  private TalonFXSimState pivotSim;
  private TalonFXSimState intakeSim;

  private final SingleJointedArmSim pivotPhysicsSim = new SingleJointedArmSim(DCMotor.getKrakenX44(1),
   1.0 / IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE,
  0.05,
   0.35,
   0, Math.toRadians(90), true, Math.toRadians(90));

  // Mechanism2d
  private final Mechanism2d intakeMech = new Mechanism2d(2, 2);
  private final MechanismRoot2d intakeRoot = intakeMech.getRoot("Pivot", 1, 1);
  private final MechanismLigament2d intakeLigament = intakeRoot.append(new MechanismLigament2d("IntakeArm", 0.5, 90));

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  intakePivotRight = new TalonFX(IntakeConstants.INTAKE_PIVOT_RIGHT_ID);
  intakePivotLeft = new TalonFX(IntakeConstants.INTAKE_PIVOT_LEFT_ID);
  
  RobotContainer.applyTalonConfigs(intakeMotor, new TalonFXConfiguration());
  RobotContainer.applyTalonConfigs(intakePivotRight, new TalonFXConfiguration());
  RobotContainer.applyTalonConfigs(intakePivotLeft, new TalonFXConfiguration());

  PIDConfigIntake = new Slot0Configs();
  PIDConfigIntake.kS = IntakeConstants.kS;
  PIDConfigIntake.kP = IntakeConstants.kP;
  PIDConfigIntake.kV = IntakeConstants.kV;
  PIDConfigIntake.kI = IntakeConstants.kI;
  PIDConfigIntake.kD = IntakeConstants.kD;

  intakeMotorConfig = new TalonFXConfiguration()
  .withMotorOutput(IntakeConstants.INTAKE_MOTOR_CONFIGS)
  .withCurrentLimits(IntakeConstants.INTAKE_CURRENT_LIMITS);

  intakePivotConfig = new TalonFXConfiguration()
  .withMotorOutput(IntakeConstants.INTAKE_PIVOT_CONFIGS)
  .withSoftwareLimitSwitch(IntakeConstants.INTAKE_PIVOT_SOFTWARE_CONFIGS)
  .withCurrentLimits(IntakeConstants.INTAKE_CURRENT_LIMITS)
  .withSlot0(PIDConfigIntake);

  RobotContainer.applyTalonConfigs(intakeMotor, intakeMotorConfig);
  RobotContainer.applyTalonConfigs(intakePivotRight, intakePivotConfig);
  RobotContainer.applyTalonConfigs(intakePivotLeft, intakePivotConfig);

  intakePivotLeft.getConfigurator().apply(PIDConfigIntake);
  intakePivotRight.getConfigurator().apply(PIDConfigIntake);
  intakePivotLeft.setPosition(0);
  intakePivotRight.setPosition(0);

  intakeRequestTurn = new PositionVoltage(0).withEnableFOC(true);

  intakePivotRight.setControl(new Follower(IntakeConstants.INTAKE_PIVOT_LEFT_ID, MotorAlignmentValue.Opposed));

  pivotSim = intakePivotLeft.getSimState();
  intakeSim = intakeMotor.getSimState();
  
  SmartDashboard.putData("Intake Mech2d", intakeMech);
  SmartDashboard.putData("Intake", this);
  }

  public void turnIntake(double angle) {
    intakeRequestTurn.withPosition(angle * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);
    intakePivotLeft.setControl(intakeRequestTurn);
  }

  public Command setIntakeSpeed(double output){
    return runOnce(() -> intakeMotor.set(output));
  }

  public Command setIntakePivotSpeed(double output){
    return runOnce(() -> intakePivotLeft.set(output));
  }

  public void intakeOut() {
      intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
      intakePivotLeft.set(IntakeConstants.INTAKE_PIVOT_SPEED);
  }

  public Command intakeOutAuto() {
      return runOnce(() -> {
        intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
        intakePivotLeft.set(IntakeConstants.INTAKE_PIVOT_SPEED);
    });
  }

  public void intakeIn() {
      intakeMotor.set(0);
      intakePivotLeft.set(-IntakeConstants.INTAKE_PIVOT_SPEED_IN);
  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void stopIntakePivot(){
    intakePivotLeft.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    pivotPhysicsSim.setInput(pivotSim.getMotorVoltage());
    pivotPhysicsSim.update(.020);

    // move the simulated motors
    pivotSim.setRawRotorPosition(Math.toDegrees(pivotPhysicsSim.getAngleRads()) * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);
    pivotSim.setRotorVelocity(Math.toDegrees(pivotPhysicsSim.getAngleRads()) * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);

    intakeLigament.setAngle(Math.toDegrees(pivotPhysicsSim.getAngleRads()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

    builder.addDoubleProperty("Pivot voltage", () -> intakePivotLeft.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("Pivot velocity", () -> intakePivotLeft.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("Pivot angle", () -> intakePivotLeft.getPosition().getValueAsDouble() / IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE, null);
    builder.addDoubleProperty("Pivot position", () -> intakePivotLeft.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Pivot current", () -> intakePivotLeft.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Pivot stator current", () -> intakePivotLeft.getStatorCurrent().getValueAsDouble(), null);

    builder.addDoubleProperty("Intake RPM", () -> intakeMotor.getVelocity().getValueAsDouble() * 60, null);
  }
}
