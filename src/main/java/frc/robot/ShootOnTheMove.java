// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class ShootOnTheMove {
    public DoubleSupplier setAngle = () -> 0.0;
    public void update(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeed, ShooterSubsystem shooter) {

        //Latency
        double latency = ShooterConstants.LATENCY_SEC;
        Translation2d futurePos = robotPose.get().getTranslation().plus(
            new Translation2d(robotSpeed.get().vxMetersPerSecond, robotSpeed.get().vyMetersPerSecond).times(latency).rotateBy(robotPose.get().getRotation())
        );

        //get Target Vecotr
        Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_HUB_GOAL : TrajectoryConstants.RED_HUB_GOAL;
        Translation2d targetVector = goalLocation.minus(futurePos);
        double distance = targetVector.getNorm();

        double idealHorizontalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(ShooterConstants.HOOD_ANGLE_SHOOT));
        double idealVerticalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(ShooterConstants.HOOD_ANGLE_SHOOT));

        //minus vectors
        Translation2d robotVelocityVector = new Translation2d(robotSpeed.get().vxMetersPerSecond, robotSpeed.get().vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        //get the angles and stuff
        double turretAngle = shotVector.getAngle().getDegrees();
        double newHorizontalSpeed = shotVector.getNorm();

        shooter.setRPM((Math.hypot(newHorizontalSpeed, idealVerticalSpeed) / (ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
        shooter.turnShooter(turretAngle);

        setAngle = () -> 15.0;
    }

    public DoubleSupplier getSetAngle() {
        return setAngle;
    }
}
