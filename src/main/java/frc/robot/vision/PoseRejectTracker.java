// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.EnumMap;

/** Add your docs here. */
public class PoseRejectTracker {
    private final EnumMap<PoseInvalidCause, Integer> counts = 
        new EnumMap<>(PoseInvalidCause.class);

    public PoseRejectTracker() {
        for (var cause : PoseInvalidCause.values())
            counts.put(cause, 0);
    }

    public void record(PoseInvalidCause cause) {
        counts.put(cause, counts.get(cause) + 1);
    }

    public EnumMap<PoseInvalidCause, Integer> getCounts() {
        return counts;
    }
}
