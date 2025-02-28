package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLFunnelConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;

public class VisionSubsystem extends SubsystemBase{
  SwerveSubsystem swerve;
  String ll_name;
  int tagid;
  double tx;
  double ty;
  double ta;
  double tv;
  double distToCamera;
  double currentPipeline;
  PIDController sideController;
  PIDController frontController;
  double frontAdjust = 0;
  double leftSideAdjust = 0;
  double rightSideAdjust = 0;
  double midSideAdjust = 0;
  double alignmentSideOffset;
  NetworkTable table;

  public VisionSubsystem(SwerveSubsystem swerve, String ll_name, Pose3d cameraOffset) {
    this.swerve = swerve;
    this.ll_name = ll_name;

    if (ll_name == "limelight-reef") {
    LimelightHelpers.setCameraPose_RobotSpace(ll_name, LLReefConstants.X_CAMERA_OFFSET, LLReefConstants.Y_CAMERA_OFFSET, LLReefConstants.Z_CAMERA_OFFSET, LLReefConstants.ROLL_CAMERA_OFFSET, 
        LLReefConstants.PITCH_CAMERA_OFFSET, LLReefConstants.YAW_CAMERA_OFFSET);
  } else {
      LimelightHelpers.setCameraPose_RobotSpace(ll_name, LLFunnelConstants.X_CAMERA_OFFSET, LLFunnelConstants.Y_CAMERA_OFFSET,
        LLFunnelConstants.Z_CAMERA_OFFSET, LLFunnelConstants.ROLL_CAMERA_OFFSET,
        LLFunnelConstants.PITCH_CAMERA_OFFSET, LLFunnelConstants.YAW_CAMERA_OFFSET);
    }

    //Setup PIDControllers for AlignToTag
    sideController = new PIDController(5, 0, 0);
    sideController.setSetpoint(0.0);
    sideController.setTolerance(ApriltagConstants.TRANSLATION_SIDE_POSE_TOLERANCE);

    frontController = new PIDController(5, 0, 0);
    frontController.setSetpoint(0.0);
    frontController.setTolerance(ApriltagConstants.TRANSLATION_FRONT_POSE_TOLERANCE);
  }
  
  @Override
  public void periodic(){
    updateLimelightTracking();
    //UpdateLocalization();
    UpdateAlignerPID();
  }
  
  public void updateLimelightTracking(){ 
    table = NetworkTableInstance.getDefault().getTable(ll_name);
      
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tv = table.getEntry("tv").getDouble(0);
    Double[] temp = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] {});
    if (temp.length == 0) {
      return;
    }
    distToCamera = temp[9];
  
    currentPipeline = table.getEntry("getpipe").getDouble(0);
  }
    
  public Pose2d getEstimatedPose() {
    double yaw = swerve.getHeading().getDegrees();
    double yawRate = swerve.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond);
    Double[] poseArray = { yaw, yawRate, 0., 0., 0., 0. };
    table.getEntry("robot_orientation_set").setDoubleArray(poseArray);

    Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] {});
    if (botPoseArray.length == 0 || botPoseArray[7] == 0) {
      return null;
    }
    Pose2d pose = new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(Units.degreesToRadians(botPoseArray[5])));
    return pose;
  }
    
  public double getPoseTimestamp() {
    Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] {});
    if (botPoseArray.length == 0) {
      return Double.NaN;
    }
    double timeStamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(botPoseArray[6]);
    return timeStamp;
  }
  
  public void setPipeline(int index){
    table.getEntry("pipeline").setNumber(index); 
  }
  
  public boolean getObjectedDetected(){
    if (tv == 1) return true;
    else return false;
  }
  public int getTagID(){
    return tagid;
  }
  public double getTX(){
    return tx;
  }
  public double getTY(){
    return ty;
  }
  public double getTA(){
    return ta;
  }
  public double getCurrentPipeline(){
    return currentPipeline;
  }
  
  public double getDistToCamera(){
    return distToCamera;
  }
  public String getName(){
    return ll_name;
  }
  public enum ReefPosition {
    LEFT,
    MIDDLE,
    RIGHT
  }
}







  /*
    public void UpdateLocalization() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

    //   double yaw = swerve.getHeading().getDegrees();
    //   double yawRate = swerve.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond);
    //   Double[] poseArray = {yaw, yawRate, 0., 0., 0., 0.};
    //   table.getEntry("robot_orientation_set").setDoubleArray(poseArray);

    //   Double[] rawFiducials = table.getEntry("rawfiducials").getDoubleArray(new Double[] { });
    //   if (rawFiducials.length == 0) {
    //     SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
    //     return;
    //   }

    //   double avgAmbig = 0;
    //   int rawFiducialsLength = rawFiducials.length / 7;
    //   for (int i = 1; i <= rawFiducialsLength; i++) {
    //     SmartDashboard.putNumber(ll_name + " TagID #" + i, rawFiducials[i * 7 - 7]);
    //     SmartDashboard.putNumber(ll_name + " TagX #" + i, rawFiducials[i * 7 - 6]);
    //     SmartDashboard.putNumber(ll_name + " TagY #" + i, rawFiducials[i * 7 - 5]);
    //     SmartDashboard.putNumber(ll_name + " Distance to tag #" + i, rawFiducials[i * 7 - 3]); // tag to camera
    //     SmartDashboard.putNumber(ll_name + " Ambiguity #" + i, rawFiducials[i * 7 - 1]);

    //     avgAmbig += rawFiducials[i * 7 - 1] / rawFiducialsLength;
    //   }

    //   SmartDashboard.putNumber("avg ambigg " + ll_name, avgAmbig);

    //   if (avgAmbig > 0.5) {
    //     SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
    //     return;
    //   }

    //   Double[] botPoseArray = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new Double[] { });
      
    //   if (botPoseArray.length == 0) {
    //     SmartDashboard.putBoolean(ll_name + " Pose Estimated ", false);
    //     return;
    //   }

    //   double timeStamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(botPoseArray[6]);
    //   Pose2d pose = new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(Units.degreesToRadians(botPoseArray[5])));
    //   swerve.addVisionReading(pose, timeStamp);

      SmartDashboard.putBoolean(ll_name + " Pose Estimated ", true);
    }
    */
    