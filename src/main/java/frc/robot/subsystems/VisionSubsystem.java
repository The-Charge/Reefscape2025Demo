package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.estimator.PoseEstimate;
import limelight.structures.AngularVelocity3d;
import limelight.structures.LimelightSettings.LEDMode;
import limelight.structures.Orientation3d;

public class VisionSubsystem extends SubsystemBase{
    SwerveSubsystem swerve;

    //private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Limelight limelight;
    LimelightPoseEstimator llPoseEstimator;

    public String ll_name;
    RawFiducial[] fiducials;

    int tagid;
    double txnc;
    double tync;
    double ta;
    double distToCamera;
    double distToRobot;
    double ambiguity;
    double currentPipeline;

    public VisionSubsystem(SwerveSubsystem swerve, String ll_name, Pose3d cameraOffset){
      this.swerve = swerve;
      this.ll_name = ll_name;
      //Setup YALL limelight object (maybe should be in initialize)
      limelight = new Limelight(ll_name);
      limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(cameraOffset).save();
      llPoseEstimator = limelight.getPoseEstimator(true);
      
    }

    @Override
    public void periodic(){
      updateLimelightTracking();
      UpdateLocalization();
    }
  
    public void updateLimelightTracking(){ 
      fiducials = LimelightHelpers.getRawFiducials(ll_name);
      for (RawFiducial fiducial : fiducials) {
          tagid = fiducial.id;                   // Tag ID
          txnc = fiducial.txnc;                  // X offset (no crosshair)
          tync = fiducial.tync;                  // Y offset (no crosshair)
          ta = fiducial.ta;                      // Target area
          distToCamera = fiducial.distToCamera;  // Distance to camera
          distToRobot = fiducial.distToRobot;    // Distance to robot
          ambiguity = fiducial.ambiguity;        // Tag pose ambiguity
    }
  }
    public void setPipeline(double index){
      NetworkTableInstance.getDefault().getTable(ll_name).getEntry("pipeline").setNumber(index);
    }

    public void adjustDriverPipeline(){
      if (currentPipeline == 1) 
        NetworkTableInstance.getDefault().getTable(ll_name).getEntry("pipeline").setNumber(0);
      else
        NetworkTableInstance.getDefault().getTable(ll_name).getEntry("pipeline").setNumber(1);
    }
   
    public void UpdateLocalization() {
      limelight.getSettings()
          .withRobotOrientation(new Orientation3d(new Rotation3d(0, 0, swerve.getHeading().getRadians()),
              new AngularVelocity3d(DegreesPerSecond.of(0),
                  DegreesPerSecond.of(0),
                  swerve.getSwerveDrive().getGyro().getYawAngularVelocity().copy())))
          .save();
      SmartDashboard.putNumber("swerve rotation", swerve.getRotation3d().getX());
      // Get the vision estimate.
      Optional<PoseEstimate> visionEstimate = llPoseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
        // poseEstimate.printPoseEstimate(); // poseEstimate does not work with multiple
        // limelights
        if (poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.4) {
          swerve.addVisionReading(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
          SmartDashboard.putNumber(ll_name + " TagX ", poseEstimate.pose.toPose2d().getX());
          SmartDashboard.putNumber(ll_name + " TagY ", poseEstimate.pose.toPose2d().getY());
          SmartDashboard.putNumber(ll_name + " Timestamp ", poseEstimate.timestampSeconds);
          SmartDashboard.putNumber(ll_name + " Rotationtag ", poseEstimate.pose.toPose2d().getRotation().getDegrees());
          SmartDashboard.putNumber(ll_name + " Distance to tag ", poseEstimate.avgTagDist);
          SmartDashboard.putBoolean(ll_name + " Pose Estimated ", true);
        } else {
          SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
        }
        SmartDashboard.putNumber(ll_name + " Tag Ambig ", poseEstimate.getMinTagAmbiguity());

      });
    }
    
    public boolean robotRotationWithinThreshold(int tagid){
      return 
        Math.abs(ApriltagConstants.TAG_POSES[tagid].getRotation().getAngle() * 180 / Math.PI - 180
        - llPoseEstimator.getPoseEstimate().get().pose.toPose2d().getRotation().getDegrees())
        < ApriltagConstants.ANGLE_POSE_TOLERANCE;
    }

  
    public boolean getObjectedDetected(){
      return LimelightHelpers.getTV(ll_name);
    }
    public int getTagID(){
      return tagid;
    }
    public double getTXNC(){
      return txnc;
    }
    public double getTYNC(){
      return tync;
    }
    public double getTA(){
      return ta;
    }
    public double getDistToRobot(){
      return distToRobot;
    }
    public double getDistToCamera(){
      return distToCamera;
    }
    public String getName(){
      return ll_name;
  }
}