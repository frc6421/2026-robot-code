// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

/** Add your docs here. */
public class SOTMTable {
    private static final InterpolatingDoubleTreeMap horizontalVelocityMap =
        new InterpolatingDoubleTreeMap();

    static {
        // key = Distance (m); value = Velocity (RPM)
        horizontalVelocityMap.put(Inches.of(55).in(Meters), 2200.0);
        horizontalVelocityMap.put(Inches.of(76).in(Meters), 2406.0);
        horizontalVelocityMap.put(Inches.of(91).in(Meters), 2543.0);
        horizontalVelocityMap.put(Inches.of(104).in(Meters), 2704.0);
        horizontalVelocityMap.put(Inches.of(132).in(Meters), 2956.0);
        horizontalVelocityMap.put(Inches.of(160).in(Meters), 3208.0);
        horizontalVelocityMap.put(Inches.of(180).in(Meters), 3437.0);
        horizontalVelocityMap.put(Inches.of(207).in(Meters), 3621.0);
    }
    /**
     * This method assumes constant angle, only use when Shooting, never when Passing
     * @param distanceFromGoal the distance from the goal target
     */
    public static double getSpeed(double distanceFromGoal) {
        return horizontalVelocityMap.get(distanceFromGoal);
    }
}
