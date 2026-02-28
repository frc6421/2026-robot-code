// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class PassOnTheMove {
    public void update(Pose2d robotPose, ChassisSpeeds robotSpeed, ShooterSubsystem shooter) {

        //Latency
        double latency = ShooterConstants.LATENCY_SEC;
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency).rotateBy(robotPose.getRotation())
        );

        //get Target Vecotr
        Translation2d goalLocation = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? TrajectoryConstants.BLUE_BUMP_O.getTranslation() : TrajectoryConstants.RED_BUMP_O.getTranslation();
        Translation2d targetVector = goalLocation.minus(futurePos);
        double distance = targetVector.getNorm();

        double idealHorizontalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.cos(Math.toRadians(shooter.getHoodAngle())); // TODO is this right?
        double idealVerticalSpeed = (SOTMTable.getSpeed(distance) / 60.0) * (ShooterConstants.WHEEL_DIAMETER * Math.PI) * Math.sin(Math.toRadians(shooter.getHoodAngle()));

        //minus vectors
        Translation2d robotVelocityVector = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVector = targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        //get the angles and stuff
        double turretAngle = shotVector.getAngle().getDegrees();
        double newHorizontalSpeed = shotVector.getNorm();

        double totalVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed);

        double ratio = Math.min(newHorizontalSpeed / totalVelocity, 1.0);
        double newHoodAngle = Math.acos(ratio);

        //shooter.setRPM((Math.hypot(newHorizontalSpeed, idealVerticalSpeed) / (ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
        shooter.turnShooter(turretAngle);
        //shooter.setHoodAngle(newHoodAngle);
    }
}
