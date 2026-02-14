// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class VisionPoseValidator {
    private static final double AMBIGUITY_LIMIT = 0.70;
    private static final double DISTANCE_LIMIT = 7.5;
    private static final double CONFIDENCE_LIMIT = 0.10;
    private static final double POSE_JUMP_LIMIT = 2.5;
    private static final double X_MAX = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark).getFieldLength();
    private static final double Y_MAX = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark).getFieldLength();

    private static Pose2d lastPose = null; 
    public static PoseInvalidCause validate(
        Pose2d pose,
        double ambiguity,
        double confidence,
        double distance
    ) {
        if (ambiguity > AMBIGUITY_LIMIT) return PoseInvalidCause.HIGH_AMBIGUITY;
        if (ambiguity > DISTANCE_LIMIT) return PoseInvalidCause.TOO_FAR;
        if (ambiguity < CONFIDENCE_LIMIT ) return PoseInvalidCause.LOW_CONFIDENCE;

        if ((pose.getX() > X_MAX) || (pose.getX() < 0.0) ||
        (pose.getY() > Y_MAX) || (pose.getY() < 0.0)) return PoseInvalidCause.OUT_OF_FIELD;

        // if(lastPose != null && pose.getTranslation().getDistance(lastPose.getTranslation()) > POSE_JUMP_LIMIT)
        // return PoseInvalidCause.POSE_JUMP;

        lastPose = pose;
        return null;
    }
}
