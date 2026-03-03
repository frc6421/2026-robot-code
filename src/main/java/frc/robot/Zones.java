// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public record Zones() {
    private static final class AreaList {
        //Area MUST START AT TOP LEFT OF AREA
        private static final class RED_ALLIANCE {
            private static final Translation2d TOP_LEFT = 
                new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters));
            
            private static final int WIDTH = (int) Inches.of(651.22 - 469.11).in(Meters);
            private static final int HEIGHT = (int) Inches.of(317.69).in(Meters); 

            // new Translation2d(Inches.of(651.22).in(Meters), Inches.of(317.69).in(Meters))
            // new Translation2d(Inches.of(651.22).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters))
        };

        private static final class BLUE_ALLIANCE {
            private static final Translation2d TOP_LEFT = 
                new Translation2d(Inches.of(0).in(Meters), Inches.of(317.69).in(Meters));

            private static final int WIDTH = (int) Inches.of(182.11).in(Meters);
            private static final int HEIGHT = (int) Inches.of(317.69).in(Meters);

            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters)),
            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(0).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(0).in(Meters), Inches.of(317.69).in(Meters))
        };

        private static final class NEUTRAL_HIGH {
            private static final Translation2d TOP_LEFT = 
                new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters));
            
            private static final int WIDTH = (int) Inches.of(469.11 - 182.11).in(Meters);
            private static final int HEIGHT = (int) Inches.of(317.69 / 2.0).in(Meters);
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters)),
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters))
           
        };

        private static final class NEUTRAL_LOW {
            private static final Translation2d TOP_LEFT = 
                new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69 / 2.0).in(Meters));
            
            private static final int WIDTH = (int) Inches.of(469.11 - 182.11).in(Meters);
            private static final int HEIGHT = (int) Inches.of(317.69 / 2.0).in(Meters);
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters)),
            // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
            // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters))
           
        };
    }
    public static Zone allianceZone = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
     ? new Zone(AreaList.BLUE_ALLIANCE.TOP_LEFT, AreaList.BLUE_ALLIANCE.WIDTH, AreaList.BLUE_ALLIANCE.HEIGHT) : 
     new Zone(AreaList.RED_ALLIANCE.TOP_LEFT, AreaList.RED_ALLIANCE.WIDTH, AreaList.RED_ALLIANCE.HEIGHT);

    public static Zone opposingZone = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
    ? new Zone(AreaList.RED_ALLIANCE.TOP_LEFT, AreaList.RED_ALLIANCE.WIDTH, AreaList.RED_ALLIANCE.HEIGHT) : 
     new Zone(AreaList.BLUE_ALLIANCE.TOP_LEFT, AreaList.BLUE_ALLIANCE.WIDTH, AreaList.BLUE_ALLIANCE.HEIGHT);

    public static Zone neutralHighZone = new Zone(AreaList.NEUTRAL_HIGH.TOP_LEFT, AreaList.NEUTRAL_HIGH.WIDTH, AreaList.NEUTRAL_HIGH.HEIGHT);
    public static Zone neutralLowZone = new Zone(AreaList.NEUTRAL_LOW.TOP_LEFT, AreaList.NEUTRAL_LOW.WIDTH, AreaList.NEUTRAL_LOW.HEIGHT);
}
