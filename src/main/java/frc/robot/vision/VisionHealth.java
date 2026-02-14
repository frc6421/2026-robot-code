// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class VisionHealth {
    private static final double DEFAULT_HEALTH = 1.0;

    private final Map<String, Double> health = new HashMap<>();

    public void update(String cam, boolean valid) {
        health.put(cam, health.getOrDefault(cam, DEFAULT_HEALTH) * 0.95 + (valid ? 0.05 : 0));
    }

    public double getHealth(String cam) {
        return health.getOrDefault(cam, DEFAULT_HEALTH);
    }
}
