// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.Rectangle;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;

/** Add your docs here. */
public class Zone {
    private double maxY = 0.0, minY = 0.0, maxX = 0.0, minX = 0.0;
   public Zone(Translation2d topLeftbound, double width, double height) {
        this.minX = topLeftbound.getX();
        this.minY = topLeftbound.getY() - height;
        this.maxX = topLeftbound.getX() + width;
        this.maxY = topLeftbound.getY();
   }

   public boolean isPointInZone(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
   }
}
