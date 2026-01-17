// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class WarriorCamera implements Sendable {

  private final PhotonCamera camera;
  private Pose2d cameraPose2d = new Pose2d();
  private List<PhotonPipelineResult> cameraResult;
  private PhotonPipelineResult latestCameraResult = new PhotonPipelineResult();
  private final PhotonPoseEstimator poseEstimator;
  Optional<EstimatedRobotPose> cameraEstimatedPose;
  private Matrix<N3, N1> standardDeviation;
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public final static class CameraConstants {
    // Camera 2
    public final static Transform3d FRONT_RIGHT_TRANSFORM3D = new Transform3d(new Translation3d(0.25, -0.24, 0.21), // Adding
                                                                                                                    // makes
                                                                                                                    // number
                                                                                                                    // bigger
        new Rotation3d(Units.degreesToRadians(-.24), Units.degreesToRadians(-12.5),
            Units.degreesToRadians(24.6)));
    // Camera 6
    // public final static Transform3d FRONT_LEFT_TRANSFORM3D = new Transform3d(new Translation3d(0.24, 0.28, 0.22),
    //     new Rotation3d(Units.degreesToRadians(-0), Units.degreesToRadians(-25.1),
    //         Units.degreesToRadians(-26.0)));
    // public final static Transform3d FRONT_RIGHT_TRANSFORM3D = new Transform3d(new Translation3d(0.34, -0.16, 0.12), // Adding
    //                                                                                                                 // makes
    //                                                                                                                 // number
    //                                                                                                                 // bigger
    //     new Rotation3d(Units.degreesToRadians(-1.0), Units.degreesToRadians(-15.6),
    //         Units.degreesToRadians(24.4)));
    // Camera 6
    public final static Transform3d FRONT_LEFT_TRANSFORM3D = new Transform3d(new Translation3d(0.24, 0.3, 0.31),
        new Rotation3d(Units.degreesToRadians(-0.6), Units.degreesToRadians(-21.9),
            Units.degreesToRadians(-26.0)));

    private final static AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    private final static double MAXIMUM_X_POSE = TAG_LAYOUT.getFieldLength();
    private final static double MAXIMUM_Y_POSE = TAG_LAYOUT.getFieldWidth();
    private final static double APRILTAG_LIMIT_METERS = 6.7;
    private final static double APRILTAG_LIMIT_METERS_AUTO = 1.0;
    private final static double APRILTAG_CLOSE_LIMIT_METERS = 0.0;
    private final static double MAXIMUM_AMBIGUITY = 0.20;
    private final static int[] BLACKLISTED_TAG_ID_LIST = { 4, 5, 14, 15 };

    private final static Matrix<N3, N1> LOW_SD = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));
    private final static Matrix<N3, N1> HIGH_SD = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(0.1));

    public static final Transform2d ODOMETRY_BLUE_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
        Inches.of(0.0).magnitude(), new Rotation2d());
    public static final Transform2d ODOMETRY_RED_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
        Inches.of(0.0).magnitude(), new Rotation2d());
  }

  /* Robot swerve drive state */
  private final NetworkTable cameraStateTable;
  private final StructPublisher<Pose3d> cameraPose;
  private final BooleanPublisher cameraReliable;

  public WarriorCamera(String cameraName, Transform3d offsets) {
    camera = new PhotonCamera(cameraName);
    cameraStateTable = inst.getTable(camera.getName() + "CameraState");
    cameraPose = cameraStateTable.getStructTopic("Pose", Pose3d.struct).publish();
    cameraReliable = cameraStateTable.getBooleanTopic("Reliable").publish();

    poseEstimator = new PhotonPoseEstimator(
        CameraConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, offsets);

    refreshData();

    SendableRegistry.add(this, camera.getName());
    SmartDashboard.putData(this);

  }

  public final void refreshData() {

    cameraResult = camera.getAllUnreadResults();
    if (!cameraResult.isEmpty()) {
      latestCameraResult = cameraResult.get(cameraResult.size() - 1);
      cameraEstimatedPose = poseEstimator.update(latestCameraResult);
      if (!cameraEstimatedPose.isEmpty()) {
        cameraPose2d = cameraEstimatedPose.get().estimatedPose.toPose2d();
        cameraPose.accept(cameraEstimatedPose.get().estimatedPose);
      }
    }
  }

  public boolean hasTarget() {
    return latestCameraResult.hasTargets();
  }

  public boolean isAmbiguousTags() {
    boolean containsAmbiguous = false;
    if (latestCameraResult.hasTargets()) {
      for (int c = 0; c < latestCameraResult.getTargets().size(); c++) {
        for (int i = 0; i < CameraConstants.BLACKLISTED_TAG_ID_LIST.length; i++) {
          if (latestCameraResult.getTargets().get(c).fiducialId == CameraConstants.BLACKLISTED_TAG_ID_LIST[i]) {
            containsAmbiguous = true;
          }
        }
        ;
      }
    }
    return containsAmbiguous;
  }

  private Distance getCameraDistance(Translation2d targetTranslation) {
    if (cameraEstimatedPose.isPresent()) {
      return Meters.of(
          cameraEstimatedPose.get().estimatedPose.toPose2d().getTranslation().getDistance(targetTranslation));
    } else {
      return Meters.of(cameraPose2d.getTranslation().getDistance(targetTranslation));
    }
  }

  public Pose2d getPose2d() {
    return cameraPose2d;
  }

  public Matrix<N3, N1> getStandardDeviation() {
    return standardDeviation;
  }

  public double getTimer() {
    return latestCameraResult.getTimestampSeconds();
  }

  public AprilTagFieldLayout getTagFieldLayout() {
    return CameraConstants.TAG_LAYOUT;
  }

  public int getBestTagId() {
    return latestCameraResult.getBestTarget().fiducialId;
  }

  public double getAmbiguity() {
    if (latestCameraResult.hasTargets()) {
      return latestCameraResult.getBestTarget().poseAmbiguity;
    }
    return 1.0;
  }

  // public double get

  public double getPitch() {
    return latestCameraResult.getBestTarget().getPitch();
  }

  public double getYaw() {
    return latestCameraResult.getBestTarget().getYaw();
  }

  public int getNumberOfTags() {
    return cameraEstimatedPose.get().targetsUsed.size();
  }

  public boolean filterOdometry() {
    refreshData();

    if (!camera.isConnected()) {
      cameraReliable.set(false);
      return false;
    } else if (!(cameraEstimatedPose.isPresent())) {
      cameraReliable.set(false);
      return false;
    }

    // Pose in Field??
    if (cameraPose2d.getX() > CameraConstants.MAXIMUM_X_POSE ||
        cameraPose2d.getY() > CameraConstants.MAXIMUM_Y_POSE ||
        cameraPose2d.getX() < 0 ||
        cameraPose2d.getY() < 0) {
      DataLogManager.log("Out of Field");
      cameraReliable.set(false);
      return false;
    }

    if (isTagReliable() && getNumberOfTags() >= 1) {
      standardDeviation = CameraConstants.LOW_SD;
    } else {
      standardDeviation = CameraConstants.HIGH_SD;
      cameraReliable.set(false);
      return false;
    }

    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.Red)) {
        cameraPose2d.plus(CameraConstants.ODOMETRY_RED_OFFSET);
      } else {
        cameraPose2d.plus(CameraConstants.ODOMETRY_BLUE_OFFSET);
      }
    }
    cameraReliable.set(true);
    return true;
  }

  public boolean isTagReliable() {

    if (latestCameraResult.hasTargets()) {
      PhotonTrackedTarget bestTarget = latestCameraResult.getBestTarget();
      int targetID = bestTarget.getFiducialId();
      Translation2d cameraTranslation2d = cameraPose2d.getTranslation();
      Translation2d targetTranslation2d = CameraConstants.TAG_LAYOUT.getTagPose(targetID).get().getTranslation()
          .toTranslation2d();

      // if (DriverStation.isAutonomous()) {
      //   if (cameraTranslation2d.getDistance(targetTranslation2d) < CameraConstants.APRILTAG_LIMIT_METERS_AUTO &&
      //       bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY && !isAmbiguousTags()) {
      //     return true;
      //   } else {
      //     return false;
      //   }
      // }

      if (cameraTranslation2d.getDistance(targetTranslation2d) < CameraConstants.APRILTAG_LIMIT_METERS
          && cameraTranslation2d.getDistance(targetTranslation2d) > CameraConstants.APRILTAG_CLOSE_LIMIT_METERS
          && bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY && !isAmbiguousTags()) {
        cameraReliable.set(true);
        return true;
      } else {
        cameraReliable.set(false);
        return false;
      }
    }
    cameraReliable.set(false);
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(camera.getName());
    builder.addBooleanProperty(camera.getName() + " has 2 targets?", () -> latestCameraResult.getTargets().size() >= 2, null);
    builder.addDoubleProperty(camera.getName() + " Ambiguity", () -> getAmbiguity(), null);
  }
}