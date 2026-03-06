// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.EnumMap;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.Data;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
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
import edu.wpi.first.networktables.StructArrayPublisher;
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
import frc.robot.subsystems.CommandSwerveDrivetrain;

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

  //Simulation
  private PhotonCameraSim cameraSim;
  private final SimCameraProperties simProp = new SimCameraProperties();

  public final static class CameraConstants {
    // side cam single mount
    // X: 11.868in
    // Y: 10.2795in
    // Z: 6.543in
    // Theta:

    //forward facing
    // X: 13.486in
    // Y: 9.374in
    // Z: 6.543in
    // Theta:

    // side cam dual mount
    // X: 9.6175in
    // Y: 10.5465in
    // Z: 6.443in
    // -.24, -.27, .10

    /*The amount each camera is off from the center of the robot */
    public final static Transform3d BACK_CAM_OFFSET = new Transform3d(new Translation3d(-.3425, .267, .19), // 1
        new Rotation3d(Units.degreesToRadians(0.5), Units.degreesToRadians(-24.5),
            Units.degreesToRadians(180)));
    public final static Transform3d LEFT_CAM_OFFSET = new Transform3d(new Translation3d(-.244, .2678, 0.022), // 2
        new Rotation3d(Units.degreesToRadians(-0.6), Units.degreesToRadians(-25.2),
            Units.degreesToRadians(91.0)));
    public final static Transform3d RIGHT_CAM_OFFSET = new Transform3d(new Translation3d(-.301, -.261, 0.15), // 3
        new Rotation3d(Units.degreesToRadians(0.5), Units.degreesToRadians(-22.8),
            Units.degreesToRadians(273)));

    private final static AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    /* All the contraints for whether we trust/don't trust a set of data*/
    private final static double MAXIMUM_X_POSE = TAG_LAYOUT.getFieldLength();
    private final static double MAXIMUM_Y_POSE = TAG_LAYOUT.getFieldWidth();
    private final static double APRILTAG_LIMIT_METERS = 6.7;
    private final static double APRILTAG_LIMIT_METERS_AUTO = 1.0;
    private final static double APRILTAG_CLOSE_LIMIT_METERS = 0.0;
    private final static double MAXIMUM_AMBIGUITY = 0.20;
    private final static int[] BLACKLISTED_TAG_ID_LIST = {};
    private final static int MINIMUM_ACCEPTED_TAGS = 1;

    /*
     * Standard Deviation constants, tells the robot how much to trust this data
     */
    private final static Matrix<N3, N1> LOW_SD = VecBuilder.fill(
      0.00001, 0.00001, Units.degreesToRadians(0.00001)); // Units of meters and radians
    private final static Matrix<N3, N1> HIGH_SD = VecBuilder.fill(
      0.9, 0.9, Units.degreesToRadians(0.9)); // Units of meters and radians

    /**
     * If there is ever a field fault, where a certain alliance ay be off by a certain amount, these offsets fix that
     */
    public static final Transform2d ODOMETRY_BLUE_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
        Inches.of(0.0).magnitude(), new Rotation2d());
    public static final Transform2d ODOMETRY_RED_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
        Inches.of(0.0).magnitude(), new Rotation2d());
  }

  /**
   *  Contains all the camera values published manally by us to network tables rather than via {@link Sendable}
   */
  private final NetworkTable cameraStateTable;
  private final StructPublisher<Pose3d> cameraPose;
  private final StructArrayPublisher<Pose3d> targetsPose;
  private final BooleanPublisher cameraReliable;

  /**
   * Holds all the possible invalid cases for Vision
   * 
   *<p><> OUT_OF_FIELD: When the camera says the robot is outside of the field
   *<p> BLACKLISTED: A Blacklisted tag was included in the calculated pose
   *<p> TAG_DISTANCE: A tag is too close or far for reliable values
   *<p> AMBIGUITY: A camera is only picking up one tag, and the ambiguity is too high
   *<p> NO_TARGETS: No targets are inside the latest camera result
   */
  private enum PoseInvalidCause {
    OUT_OF_FIELD,
    BLACKLISTED,
    TAG_DISTANCE,
    AMBIGUITY,
    NO_TARGETS
  }

  /**A unique data type, one that has a number correlated to each unique case in {@link PoseInvalidCause} */
  private EnumMap<PoseInvalidCause, Integer> invalidCounter = new EnumMap<>(PoseInvalidCause.class);

  public WarriorCamera(String cameraName, Transform3d offsets) {
    camera = new PhotonCamera(cameraName);

    /*Initialize all data logging, publishing data mans it is sent to NetworkTables */
    cameraStateTable = inst.getTable(camera.getName() + "CameraState");
    cameraPose = cameraStateTable.getStructTopic("Pose", Pose3d.struct).publish();
    targetsPose = cameraStateTable.getStructArrayTopic("Targets Pose", Pose3d.struct).publish();
    cameraReliable = cameraStateTable.getBooleanTopic("Reliable").publish();

    /*Adds all of the unique cases in PoseInvalidCause and sets their values to 0 */
    for (PoseInvalidCause cause : PoseInvalidCause.values()) {
      invalidCounter.put(cause, 0);
    }
    /*Simulation */
    simProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
    simProp.setCalibError(1.1, .08);
    simProp.setFPS(20);
    simProp.setAvgLatencyMs(35);
    simProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, simProp);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

    /*Pose estimators allow raw camera data to turn into usable Pose data */
    poseEstimator = new PhotonPoseEstimator(
        CameraConstants.TAG_LAYOUT, offsets);

    /*Refresh all the data for the first time, ensures everything gets set */
    refreshData();
    
    /*This section is more logging, these lines add WarriorCamera to SmartDashboard and Sendable
     * note the usage of the term this, which allows all sendable values in the class to be enclosed within this new tab
     */
    SendableRegistry.add(this, camera.getName());
    SmartDashboard.putData(this);

  }

  /**
   * Refreshes all the data for the cameras
   * Should only be called once per robot cycle (we call this in another method inside {@link CommandSwerveDrivetrain})
   * When certain camera values are {@link Optional}, always ensure that there is a check using isEmpty(),
   * otherwise you can get null pointers
   */
  public final void refreshData() {
    cameraResult = camera.getAllUnreadResults();
    if (!cameraResult.isEmpty()) {
      latestCameraResult = cameraResult.get(cameraResult.size() - 1);
      cameraEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(latestCameraResult);
      if (!cameraEstimatedPose.isEmpty()) {
        cameraPose2d = cameraEstimatedPose.get().estimatedPose.toPose2d();
        //Adds the pose to NetworkTables
        cameraPose.accept(cameraEstimatedPose.get().estimatedPose);
        setTargetsPoseList();
      }
    }
    //chaneg the values to the error logging
    refreshInvalid();
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
            recordInvalid(PoseInvalidCause.BLACKLISTED);
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

  public PhotonCameraSim getSimCam() {
    return cameraSim;
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
    if (!cameraEstimatedPose.isEmpty()) {
    return cameraEstimatedPose.get().targetsUsed.size();
    } else {
      return 0;
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return latestCameraResult;
  }

  /**In an effort to track what targets we are viewing, we use this method.
   * This is an efficient but confusing way to do this,
   * the same result could be accomplished using if statements and for loops. 
   * Once the method finds all targets, they are added to NetworkTables
   */
  public void setTargetsPoseList(){
    List<PhotonTrackedTarget> targets = latestCameraResult.getTargets();
    /*
     * There is an array poses which is taking data from a map of targets.
     * we use a placeholder variable target to cycle through all unique tags on the field,
     * if a certain tag has no value in our poses, it is null, so we filter out all values with null
     * by using another placeholder variable p to cycle throgh all the poses
     * after getting the list of real poses, we send it to NetworkTables
     */
    Pose3d[] poses = targets.stream()
        .map(target -> CameraConstants.TAG_LAYOUT
          .getTagPose(target.getFiducialId())
          .orElse(null))
        .filter(p -> p != null)
        .toArray(Pose3d[]::new);
    //Publish target poses
    targetsPose.set(poses);
  
}
  /**
   * Using constants created above, we check the data from the camera, and determine its validity
   * @return true if  the pose is valid <p> false if the pose is not
   */
  public boolean filterOdometry() {
    //ensure data is fresh, this is the only place where refreshData is called each cycle (besides initialization)
    refreshData();

    /*Is the camera connected, or do we have a pose? */
    if (!camera.isConnected()) {
      // cameraReliable is a value we pulish to NetworkTables to know if we are getting valid poses or not
      cameraReliable.set(false);
      return false;
    } else if (!(cameraEstimatedPose.isPresent())) {
      cameraReliable.set(false);
      return false;
    }

    /*Pose in Field? */
    if (cameraPose2d.getX() > CameraConstants.MAXIMUM_X_POSE ||
        cameraPose2d.getY() > CameraConstants.MAXIMUM_Y_POSE ||
        cameraPose2d.getX() < 0 ||
        cameraPose2d.getY() < 0) {
      recordInvalid(PoseInvalidCause.OUT_OF_FIELD);
      cameraReliable.set(false);
      return false;
    }

    /*Are the tags used reliable and is there enough tags? */
    if (isTagReliable() && getNumberOfTags() >= CameraConstants.MINIMUM_ACCEPTED_TAGS) {
      standardDeviation = CameraConstants.LOW_SD;
    } else {
      //Invalid data in terms of concistency is not thrown out, it is rather trusted far less
      standardDeviation = CameraConstants.HIGH_SD;
      cameraReliable.set(false);
      return false;
    }

    /*Apply alliance offsets if there are any */
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.Red)) {
        cameraPose2d.plus(CameraConstants.ODOMETRY_RED_OFFSET);
      } else {
        cameraPose2d.plus(CameraConstants.ODOMETRY_BLUE_OFFSET);
      }
    }

    /*If the code reahes this point, the pose is valid */
    cameraReliable.set(true);
    return true;
  }
  /**
   * Checks the distance from robot to tags and ambiguity of the tags to determine trust level of data
   * @return true if all conditions pass, otherwise false
   */
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

      /*
       * Rather than repeat the same code twice, make an instance variable (invalidCause) 
       * to allow us to check for specific conditions and help debug
       */
      PoseInvalidCause invalidCause = null;
      if (cameraTranslation2d.getDistance(targetTranslation2d) > CameraConstants.APRILTAG_LIMIT_METERS
          || cameraTranslation2d.getDistance(targetTranslation2d) < CameraConstants.APRILTAG_CLOSE_LIMIT_METERS) {
            invalidCause = PoseInvalidCause.TAG_DISTANCE;
          }
       else if (bestTarget.getPoseAmbiguity() > CameraConstants.MAXIMUM_AMBIGUITY && isAmbiguousTags()) {
        invalidCause = PoseInvalidCause.AMBIGUITY;
      } 
      
      // invalidCause being null implies there are no errors
      if (invalidCause != null) {
        cameraReliable.set(false);
        recordInvalid(invalidCause);
        return false;
      }
        cameraReliable.set(true);
        return true;
      // if (cameraTranslation2d.getDistance(targetTranslation2d) < CameraConstants.APRILTAG_LIMIT_METERS
      //     && cameraTranslation2d.getDistance(targetTranslation2d) > CameraConstants.APRILTAG_CLOSE_LIMIT_METERS
      //     && bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY && !isAmbiguousTags()) {
      //   cameraReliable.set(true);
      //   return true;
      // } else {
      //   cameraReliable.set(false);
      //   recordInvalid(PoseInvalidCause.UNRELIABLE);
      //   return false;
      // }
    }
    cameraReliable.set(false);
    recordInvalid(PoseInvalidCause.NO_TARGETS);
    return false;
  }

  /**
   * Incrememnts the value for a specific camera error when called
   * @param cause - The specific error that was triggered
   */
  private void recordInvalid(PoseInvalidCause cause) {
    invalidCounter.put(cause, invalidCounter.get(cause) + 1);
  }

  /** Update network tables with the new values of the errors logged, 
   * cycling through each unique cause inside the enum */
  private void refreshInvalid() {
    for (PoseInvalidCause cause : PoseInvalidCause.values()) {
      cameraStateTable.getEntry(cause.name()).setInteger(invalidCounter.get(cause));
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(camera.getName());
    builder.addBooleanProperty(camera.getName() + " has 2 targets?",
     () -> latestCameraResult.getTargets().size() >= 2, null);
    builder.addDoubleProperty(camera.getName() + " Ambiguity", () -> getAmbiguity(), null);
  }
}