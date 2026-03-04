// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

/** Add your docs here. */
public class POTMTable {
    private static final InterpolatingDoubleTreeMap velocityMapRPM =
        new InterpolatingDoubleTreeMap();

    static {
        // key = Distance (m); value = Velocity (RPM)
        velocityMapRPM.put(Inches.of(167).in(Meters), 4036.0);
        velocityMapRPM.put(Inches.of(239).in(Meters), 4901.0);
        velocityMapRPM.put(Inches.of(400).in(Meters), 5341.0);
    }
    /**
     * This method assumes constant angle, only use when Shooting, never when Passing
     * @param distanceFromGoal the distance from the goal target
     */
    public static double getSpeed(double distanceFromGoal) {
        return velocityMapRPM.get(distanceFromGoal);
    }
}
