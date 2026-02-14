// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public record TagObservation(
    int id,
    Pose2d fieldPose,
    Pose2d fieldEstimate,
    double ambiguity,
    double distanceMeters
) {}
