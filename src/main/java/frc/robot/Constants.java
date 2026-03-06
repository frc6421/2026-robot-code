// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double DRIVE_SLEW_RATE = 10.0;
  }

  public static class AlignConstants {
    public static double ALIGN_P = 2.0;
    public static double MAX_VELOCITY = 2.5;
    public static double MAX_ACCELERATION = 4.5;
    public static double VISION_ERROR_ACCOUNT = 0.15;

    public static double MAX_POSITION_ERROR_METERS = 0.08;
    public static double MAX_DEGREES_ERROR = 0.5;

    public static double MAX_POSITION_ERROR_METERS_AUTO = 0.2;

  }

  public static class AutoConstants {
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4.0; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0; // TODO update value
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 2 * Math.PI; // TODO update value
    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI; // TODO update value

    public static final double X_DRIVE_P = 2.05;
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0.1;

    public static final double Y_DRIVE_P = 2.05;
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0.1;

    public static final double THETA_P = 5.0;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;
  }

  public static class TrajectoryConstants {

// robot dimensions: intake-shooter = 36.125, side-side = 31.125
// facing intake forward on blue: 0 degrees
// facing high y values: 90 degrees

// outpost is on the right side (Drive Perspective)

// all pose2d are to be finalized

    public static final Pose2d BLUE_OUTPOST = new Pose2d (new Translation2d(Inches.of(0.5), Inches.of(25.62)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_OUTPOST = new Pose2d (new Translation2d(Inches.of(649.62), Inches.of(291.02)), new Rotation2d(Degrees.of(0)));
    
    //public static final Pose2d BLUE_TOWER_DEPOT = new Pose2d (new Translation2d(Inches.of(45.0), Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    //public static final Pose2d RED_TOWER_DEPOT = new Pose2d (new Translation2d(Inches.of(605.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    //public static final Pose2d BLUE_TOWER_OUTPOST = new Pose2d (new Translation2d(Inches.of(45.0), Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    //public static final Pose2d RED_TOWER_OUTPOST = new Pose2d (new Translation2d(Inches.of(605.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    //public static final Pose2d BLUE_ALLIANCE = new Pose2d (new Translation2d(Inches.of(60.0).in(Meters), Inches.of(146.86).in(Meters)), new Rotation2d(Degrees.of(180)));
    //public static final Pose2d RED_ALLIANCE = new Pose2d (new Translation2d(Inches.of(590.12).in(Meters), Inches.of(169.78).in(Meters)), new Rotation2d(Degrees.of(360)));
   
    //public static final Pose2d BLUE_DEPOT_SIDE = new Pose2d (new Translation2d(Inches.of(60.0), Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    //public static final Pose2d RED_DEPOT_SIDE = new Pose2d (new Translation2d(Inches.of(590.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    //public static final Pose2d BLUE_OUTPOST_SIDE = new Pose2d (new Translation2d(Inches.of(60.0), Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    //public static final Pose2d RED_OUTPOST_SIDE = new Pose2d (new Translation2d(Inches.of(590.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    public static final Pose2d NEUTRAL_BO = new Pose2d (new Translation2d(Inches.of(289.11), Inches.of(67.37)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d NEUTRAL_BD = new Pose2d (new Translation2d(Inches.of(289.11), Inches.of(249.27)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d NEUTRAL_RO = new Pose2d (new Translation2d(Inches.of(361.01), Inches.of(249.27)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d NEUTRAL_RD = new Pose2d (new Translation2d(Inches.of(361.01), Inches.of(67.37)), new Rotation2d(Degrees.of(180)));

    public static final Pose2d RD_FAR_EDGE = new Pose2d (new Translation2d(Inches.of(325.06), Inches.of(85.0)), new Rotation2d(Degrees.of(90))); 
    public static final Pose2d NEUTRAL_RD_EDGE = new Pose2d (new Translation2d(Inches.of(361.01), Inches.of(100.0)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d NEUTRAL_RO_EDGE = new Pose2d (new Translation2d(Inches.of(361.01), Inches.of(220)), new Rotation2d(Degrees.of(0)));


    public static final Pose2d CENTER_FACE_RED = new Pose2d (new Translation2d(Inches.of(325.06), Inches.of(158.32)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d CENTER_FACE_BLUE = new Pose2d (new Translation2d(Inches.of(325.06), Inches.of(158.32)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d CENTER_FACE_BORD = new Pose2d (new Translation2d(Inches.of(325.06), Inches.of(158.32)), new Rotation2d(Degrees.of(270)));
    public static final Pose2d CENTER_FACE_BDRO = new Pose2d (new Translation2d(Inches.of(325.06), Inches.of(158.32)), new Rotation2d(Degrees.of(90)));

    public static final Pose2d BLUE_BUMP_O_L = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(98.06)), new Rotation2d(Degrees.of(315)));
    public static final Pose2d BLUE_BUMP_D_L = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(218.58)), new Rotation2d(Degrees.of(45)));
    public static final Pose2d RED_BUMP_O_L = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(218.58)), new Rotation2d(Degrees.of(135)));
    public static final Pose2d RED_BUMP_D_L = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(98.06)), new Rotation2d(Degrees.of(225)));
  
    public static final Pose2d BLUE_BUMP_O_R = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(98.06)), new Rotation2d(Degrees.of(225)));
    public static final Pose2d BLUE_BUMP_D_R = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(218.58)), new Rotation2d(Degrees.of(135)));
    public static final Pose2d RED_BUMP_O_R = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(218.58)), new Rotation2d(Degrees.of(45)));
    public static final Pose2d RED_BUMP_D_R = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(98.06)), new Rotation2d(Degrees.of(315)));
    
    public static final Pose2d RED_CLIMB_DEPOT_FINAL = new Pose2d (new Translation2d(Inches.of(610), Inches.of(134)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_CLIMB_DEPOT = new Pose2d (new Translation2d(Inches.of(619), Inches.of(134)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_CLIMB_DEPOT_OFFSET = new Pose2d (new Translation2d(Inches.of(585.12), Inches.of(130)), new Rotation2d(Degrees.of(0)));

    public static final Translation2d RED_HUB_GOAL = new Translation2d(Inches.of(468.565), Inches.of(158.32)); //wrong x values, formerly 414.785
    public static final Translation2d BLUE_HUB_GOAL = new Translation2d(Inches.of(181.555), Inches.of(158.32)); //wrong x values, formerly 235.335

    public static final Pose2d RED_DEPOT_SCORE = new Pose2d (new Translation2d(Inches.of(550), Inches.of(98)), new Rotation2d(Degrees.of(315)));
    public static final Pose2d RED_OUTPOST_SCORE = new Pose2d (new Translation2d(Inches.of(550), Inches.of(218)), new Rotation2d(Degrees.of(45)));

    public static final Pose2d RED_D_WALL = new Pose2d (new Translation2d(Inches.of(570), Inches.of(35)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d RED_D_TRENCH = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(24.5)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_O_TRENCH = new Pose2d (new Translation2d(Inches.of(468.56), Inches.of(292.5)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_D_TRENCH = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(292.5)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_O_TRENCH = new Pose2d (new Translation2d(Inches.of(181.56), Inches.of(24.5)), new Rotation2d(Degrees.of(0)));

    //public static final Pose2d TEST = new Pose2d (new Translation2d(Inches.of(570), Inches.of(98)), new Rotation2d(Degrees.of(90)));

  }

  public static class ClimbPositions {
    public static final double L1_INCHES = 6.9;
    public static final double MATCH_START_INCHES = 0;
  }

  public static class IntakePositions {
    public static final double INTAKE_RETRACTED_DEGREES = 0;
    public static final double INTAKE_EXTENDED_DEGREES = 90;
    public static final double INTAKE_SPEED = 0.62;
    public static final double INTAKE_PIVOT_SPEED = 0.25;
  }

  public static class TransitionConstants {
    public static final double TRANSITION_HOPPER_SPEED = 0.25;
    public static final double TRANSITION_SHOOTER_SPEED = 1.0;
  }

  public static class ShooterConstants {
    public static final double SHOOTER_RPM = 3500;
    public static final double ACTUATOR_PASSING = 0.38;
    public static final double ACTUATOR_SHOOTING = 0.02;

    //SOTM
    public static final double LATENCY_SEC = 0.10;
    public static final double WHEEL_DIAMETER = 0.1016; // METERS
    public static final double HOOD_ANGLE_SHOOT = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_NEU_CLOSE = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_NEU_FAR = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_OPP_ALL = 55.0; // DEGREES
    public static final double HOOD_ANGLE_START = 40.0;
  }

  public static class VisionConstants {

      public final static Transform3d BACK_CAM_TRANSFORM = new Transform3d(new Translation3d(-.3425, .267, .19), // 1
        new Rotation3d(Units.degreesToRadians(0.5), Units.degreesToRadians(-24.5),
            Units.degreesToRadians(180)));
    public final static Transform3d LEFT_CAM_TRANSFORM = new Transform3d(new Translation3d(-.244, .2678, 0.022), // 2
        new Rotation3d(Units.degreesToRadians(-0.6), Units.degreesToRadians(-25.2),
            Units.degreesToRadians(91.0)));
    public final static Transform3d RIGHT_CAM_TRANSFORM = new Transform3d(new Translation3d(-.301, -.261, 0.15), // 3
        new Rotation3d(Units.degreesToRadians(0.5), Units.degreesToRadians(-22.8),
            Units.degreesToRadians(273)));

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);
  }
}
