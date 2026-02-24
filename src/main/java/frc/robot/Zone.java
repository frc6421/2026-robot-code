// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.awt.geom.Path2D;

/** Add your docs here. */
public class Zone {
    private final Path2D zone;
   public Zone(Pose2d... bound) {
        zone = new Path2D.Double();
        for (Pose2d pose : bound) {
            zone.moveTo(pose.getX(), pose.getY());
        }
   }

   public boolean isPointInZone(Pose2d pose) {
    return zone.contains(pose.getX(), pose.getY());
   }
}
