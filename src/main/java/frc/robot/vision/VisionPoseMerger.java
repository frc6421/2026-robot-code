// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class VisionPoseMerger {
    public static VisionPoseEstimate merge(List<VisionPoseEstimate> poses) {
        if(poses.isEmpty()) return null;

        double totalWeight = 0.0;
        double x = 0.0, y = 0.0, theta = 0.0;
        double newestTime = 0.0;

        for (var p : poses) {
            double w = p.confidence();
            totalWeight += w;
            x += p.pose().getX() * w;
            y += p.pose().getY() * w;
            theta += p.pose().getRotation().getRadians() * w;
        }

        Pose2d fused = new Pose2d(
            x / totalWeight,
            y / totalWeight, 
            new Rotation2d(theta / totalWeight)
        );

        double avgConfidence = poses.stream()
        .mapToDouble(VisionPoseEstimate::confidence)
        .average().orElse(0.2);

        return new VisionPoseEstimate(
        fused,
        avgConfidence,
        VisionConfidenceScorer.stdDevs(avgConfidence),
        avgConfidence,
        "Fused"
        );
    }
}
