package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.structures.LimelightSettings.LEDMode;

public class VisionSubsystem extends SubsystemBase{
    SwerveSubsystem swerve;

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

    PIDController sideController;
    PIDController frontController;
    double frontAdjust = 0;
    double leftSideAdjust = 0;
    double rightSideAdjust = 0;
    double midSideAdjust = 0;
    double alignmentSideOffset;

    public VisionSubsystem(SwerveSubsystem swerve, String ll_name, Pose3d cameraOffset){
      this.swerve = swerve;
      this.ll_name = ll_name;

      //Setup YALL limelight object (maybe should be in initialize)
      limelight = new Limelight(ll_name);
      NetworkTableInstance.getDefault().getTable(ll_name).getEntry("pipeline").setNumber(1);
      limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(cameraOffset).save();
      llPoseEstimator = limelight.getPoseEstimator(true);

      //Setup PIDControllers for AlignToTag
      sideController = new PIDController(.4, 0.001, 0.0001);
      sideController.setSetpoint(0.0);
      sideController.setTolerance(ApriltagConstants.TRANSLATION_SIDE_POSE_TOLERANCE);
      
      frontController = new PIDController(.4, 0.001, 0.0001);
      frontController.setSetpoint(0.0);
      frontController.setTolerance(ApriltagConstants.TRANSLATION_FRONT_POSE_TOLERANCE);
    }

    @Override
    public void periodic(){
      updateLimelightTracking();
      UpdateLocalization();
      UpdateAlignerPID();
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
    
    public Pose2d getEstimatedPose() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

      Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] {});

      if (botPoseArray.length == 0 || botPoseArray[7] == 0) {
        return null;
      }

      Pose2d pose = new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(Units.degreesToRadians(botPoseArray[5])));

      return pose;
    }
    
    public double getPoseTimestamp() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

      Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] {});

      if (botPoseArray.length == 0) {
        return Double.NaN;
      }

      double timeStamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(botPoseArray[6]);

      return timeStamp;
    }
   
    public void UpdateLocalization() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

      double yaw = swerve.getHeading().getDegrees();
      double yawRate = swerve.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond);
      Double[] poseArray = {yaw, yawRate, 0., 0., 0., 0.};
      table.getEntry("robot_orientation_set").setDoubleArray(poseArray);

      Double[] rawFiducials = table.getEntry("rawfiducials").getDoubleArray(new Double[] { });
      if (rawFiducials.length == 0) {
        SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
        return;
      }

      double avgAmbig = 0;
      int rawFiducialsLength = rawFiducials.length / 7;
      for (int i = 1; i <= rawFiducialsLength; i++) {
        SmartDashboard.putNumber(ll_name + " TagID #" + i, rawFiducials[i * 7 - 7]);
        SmartDashboard.putNumber(ll_name + " TagX #" + i, rawFiducials[i * 7 - 6]);
        SmartDashboard.putNumber(ll_name + " TagY #" + i, rawFiducials[i * 7 - 5]);
        SmartDashboard.putNumber(ll_name + " Distance to tag #" + i, rawFiducials[i * 7 - 3]); // tag to camera
        SmartDashboard.putNumber(ll_name + " Ambiguity #" + i, rawFiducials[i * 7 - 1]);

        avgAmbig += rawFiducials[i * 7 - 1] / rawFiducialsLength;
      }

      SmartDashboard.putNumber("avg ambigg " + ll_name, avgAmbig);

      if (avgAmbig > 0.5) {
        SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
        return;
      }

      Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] { });
      
      if (botPoseArray.length == 0) {
        SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
        return;
      }

      double timeStamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(botPoseArray[6]);
      Pose2d pose = new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(Units.degreesToRadians(botPoseArray[5])));
      // swerve.addVisionReading(pose, timeStamp);

      SmartDashboard.putBoolean(ll_name + " Pose Estimated ", true);
    }

    public void UpdateAlignerPID(){
      sideController.reset();
      frontController.reset();
      leftSideAdjust = MathUtil.clamp(sideController.calculate(txnc + ApriltagConstants.LEFT_ALIGN_OFFSET,0), -0.2, 0.2);
      midSideAdjust = MathUtil.clamp(sideController.calculate(txnc,0), -0.2, 0.2);
      rightSideAdjust = MathUtil.clamp(sideController.calculate(txnc + ApriltagConstants.RIGHT_ALIGN_OFFSET,0), -0.2, 0.2);
      frontAdjust = MathUtil.clamp(frontController.calculate(distToRobot, ApriltagConstants.APRILTAG_POSE_OFFSET), -0.5, 0.5);
    }

    public boolean robotRotationWithinThreshold(int tagid){
      return 
        Math.abs(ApriltagConstants.TAG_POSES[tagid].getRotation().getAngle() * 180 / Math.PI - 180
        - llPoseEstimator.getPoseEstimate().get().pose.toPose2d().getRotation().getDegrees())
        < ApriltagConstants.ANGLE_POSE_TOLERANCE;
    }

  
    public boolean getObjectedDetected(){
      if (txnc != 0.0) return true;
      else return false;
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

    public double getRobotPoseTagSpace(){
      return LimelightHelpers.getBotPose3d_TargetSpace(ll_name).toPose2d().getX();
    }
    public double getSideAdjustment(double adjustDirection){
      if (adjustDirection == -1) return leftSideAdjust;
      else if (adjustDirection == 0) return midSideAdjust;
      else if (adjustDirection == 1) return rightSideAdjust;
      return 0;
    }
    public double getFrontAdjustment(){
        return frontAdjust;
    }


}