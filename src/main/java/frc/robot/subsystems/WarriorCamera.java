// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.PoseInvalidCause;
import frc.robot.vision.PoseRejectTracker;
import frc.robot.vision.TagObservation;
import frc.robot.vision.VisionCameraConfig;
import frc.robot.vision.VisionConfidenceScorer;
import frc.robot.vision.VisionHealth;
import frc.robot.vision.VisionPoseEstimate;
import frc.robot.vision.VisionPoseMerger;
import frc.robot.vision.VisionPoseValidator;

public class WarriorCamera extends SubsystemBase {

  private final List<VisionCameraConfig> cameras = List.of(
    new VisionCameraConfig("BackCam", Constants.VisionConstants.BACK_CAM_TRANSFORM),
    new VisionCameraConfig("LeftCam", Constants.VisionConstants.LEFT_CAM_TRANSFORM),
    new VisionCameraConfig("RightCam", Constants.VisionConstants.RIGHT_CAM_TRANSFORM)
  );

  private final Map<String, PhotonCameraSim> simCams = new HashMap<>();
  private final SimCameraProperties simProp = new SimCameraProperties();

  private final Map<String, PhotonCamera> photonCams = new HashMap<>();
  private final PoseRejectTracker rejectTracker = new PoseRejectTracker();
  private final VisionHealth healthMonitor = new VisionHealth();
  private CommandSwerveDrivetrain drivetrain;

  private VisionPoseEstimate latestEstimate;

  //Sim
  private VisionSystemSim visionSim;
  private Field2d simField;
  /** Creates a new WarriorCamera. */
  public WarriorCamera(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    cameras.forEach(config ->
      photonCams.put(config.name(), new PhotonCamera(config.name())));

    if (RobotBase.isSimulation()) {
      simField = new Field2d();
      SmartDashboard.putData("SimField", simField);

      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.VisionConstants.FIELD_LAYOUT);

      simProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
      simProp.setCalibError(0.1, 0.1);
      simProp.setFPS(30);
      simProp.setAvgLatencyMs(15);
      simProp.setLatencyStdDevMs(5);

      for (var config : cameras) {

        var camSim = new PhotonCameraSim(
          photonCams.get(config.name()),
          simProp);

        camSim.enableDrawWireframe(true);

        visionSim.addCamera(camSim, config.robotToCamera());
        simCams.put(config.name(), camSim);
      }
    }
  }

  private void publishDebug(
    List<VisionPoseEstimate> cameraPoses,
    VisionPoseEstimate fused) {
      var table = NetworkTableInstance.getDefault().getTable("Vision");

      for (var pose : cameraPoses) {
        table.getEntry("CameraPose/" + pose.cameraName())
          .setDoubleArray(new double[] {
            pose.pose().getX(),
            pose.pose().getY(),
            pose.pose().getRotation().getRadians()
          });

        table.getEntry("Confidence/" + pose.cameraName())
          .setDouble(pose.confidence());
      }

      if (fused != null) {
        table.getEntry("FusedPose").setDoubleArray(new double[] {
          fused.pose().getX(),
          fused.pose().getY(),
          fused.pose().getRotation().getRadians()
        });
      }

      for (var entry : rejectTracker.getCounts().entrySet()) {
        table.getEntry("Reject/" + entry.getKey().name())
            .setInteger(entry.getValue());
      }
    }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      Pose2d simPose = drivetrain.getSimPose();

      visionSim.update(simPose);
      simField.setRobotPose(simPose);
    }
    List<VisionPoseEstimate> validPoses = new ArrayList<>();

    for (var config : cameras) {
      var cam = photonCams.get(config.name());
      var resultList = cam.getAllUnreadResults();
      if (resultList.isEmpty()) continue;
      var result = resultList.get(resultList.size() - 1);

      if (!result.hasTargets()) {
        rejectTracker.record(PoseInvalidCause.NO_TARGETS);
        healthMonitor.update(config.name(), false);
        continue;
      }

      var target = result.getBestTarget();
      double ambiguity = target.getPoseAmbiguity();
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();

      Pose2d pose = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        Constants.VisionConstants.FIELD_LAYOUT.getTagPose(target.fiducialId).get(),
        config.robotToCamera()).toPose2d();

      double confidence = VisionConfidenceScorer.compute(
        result.getTargets().size(),
        ambiguity,
        distance);

      PoseInvalidCause cause = VisionPoseValidator.validate(
        pose, ambiguity, confidence, distance);

      if (cause != null) {
        rejectTracker.record(cause);
        healthMonitor.update(config.name(), false);
        continue;
      }

      List<Integer> tagIDs = new ArrayList<>();
      for (var indexedTarget : result.getTargets()) {
        tagIDs.add(target.getFiducialId());

        var tagPose = Constants.VisionConstants.FIELD_LAYOUT.getTagPose(indexedTarget.fiducialId);
        
        if (tagPose.isPresent()) {
          List<TagObservation> tags = extractTagObservations(
          result,
          config.robotToCamera());

          logTagList(config.name(), tags);
        }
      }

      validPoses.add(new VisionPoseEstimate(
        pose,
        confidence,
        VisionConfidenceScorer.stdDevs(confidence),
        result.getTimestampSeconds(),
        config.name()));

      healthMonitor.update(config.name(), true);
    }

    latestEstimate = VisionPoseMerger.merge(validPoses);
    injectVisionMeasurement(latestEstimate);
    publishDebug(validPoses, latestEstimate);
  }

  public VisionPoseEstimate getLatestEstimate() {
    return latestEstimate;
  }

  private void injectVisionMeasurement(VisionPoseEstimate estimate) {
    if (estimate == null) return;
    drivetrain.setVisionMeasurementStdDevs(estimate.stdDevs());
    drivetrain.addVisionMeasurement(
      estimate.pose(),
      Utils.fpgaToCurrentTime(estimate.timestamp()),
      estimate.stdDevs());
  }

  private List<TagObservation> extractTagObservations(
    PhotonPipelineResult result,
    Transform3d robotToCamera
  ) {
    List<TagObservation> tags = new ArrayList<>();

    for (var target : result.getTargets()) {
      int id = target.fiducialId;

      var tagPoseOptional =
        Constants.VisionConstants.FIELD_LAYOUT.getTagPose(id);
      
      if (tagPoseOptional.isEmpty()) continue;

      Pose2d fieldPose = 
        tagPoseOptional.get().toPose2d();

      Pose2d robotEstimate = 
        PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),
          Constants.VisionConstants.FIELD_LAYOUT.getTagPose(target.fiducialId).get(),
          robotToCamera).toPose2d();

      double distance = 
        target.getBestCameraToTarget()
          .getTranslation()
          .getNorm();
      
      tags.add(new TagObservation(
        id,
        fieldPose,
        robotEstimate,
        target.poseAmbiguity,
        distance));
    }

    return tags;
  }

  private void logTagList(
    String cameraName,
    List<TagObservation> tags
  ) {
    var table = NetworkTableInstance.getDefault()
      .getTable("Vision/AllTags" + cameraName);
    
    long[] ids = new long[tags.size()];
    double[] fieldPoseArray = new double[tags.size() * 3];
    double[] robotPoseArray = new double[tags.size() * 3];
    double[] ambiguites = new double[tags.size()];
    double[] distances = new double[tags.size()];
    
    for (int i = 0; i < tags.size(); i++) {

      TagObservation t = tags.get(i);

      ids[i] = t.id();

      fieldPoseArray[i*3] = t.fieldPose().getX();
      fieldPoseArray[i*3 + 1] = t.fieldPose().getY();
      fieldPoseArray[i*3 + 2] = t.fieldPose().getRotation().getRadians();

      robotPoseArray[i*3] = t.fieldEstimate().getX();
      robotPoseArray[i*3 + 1] = t.fieldEstimate().getY();
      robotPoseArray[i*3 + 2] = t.fieldEstimate().getRotation().getRadians();

      ambiguites[i] = t.ambiguity();
      distances[i] = t.distanceMeters();
    }

    table.getEntry("IDs").setIntegerArray(ids);
    table.getEntry("FieldPoses").setDoubleArray(fieldPoseArray);
    table.getEntry("RobotEstimates").setDoubleArray(robotPoseArray);
    table.getEntry("Ambiguities").setDoubleArray(ambiguites);
    table.getEntry("Distances").setDoubleArray(distances);
  }
}
