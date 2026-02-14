// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionConfidenceScorer {
    private static final double TRUSTED_TAG_AMOUNT = 2.0;
    private static final double TRUSTED_TAG_DISTANCE = 7.0;
    private static final double XY_STD_DEVS = 0.35;
    private static final double THETA_STD_DEVS = 0.6;
    public static double compute(
        int tagCount,
        double ambiguity,
        double distanceMeters
) {
    double score = 1.0;
    
    score *= Math.min(1.0, tagCount / TRUSTED_TAG_AMOUNT);
    score *= (1.0 - ambiguity);
    score *= Math.max(0.2, 1.0 - distanceMeters / TRUSTED_TAG_DISTANCE);

    return Math.max(0.05, Math.max(score, 1.0));
}

public static Matrix<N3, N1> stdDevs(double confidence) {
    return VecBuilder.fill(
        XY_STD_DEVS / confidence,
        XY_STD_DEVS / confidence,
        THETA_STD_DEVS / confidence
    );

}
}