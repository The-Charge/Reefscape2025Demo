package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  String ll_name;

  public VisionSubsystem(String ll_name, Pose3d cameraOffset) {
    this.ll_name = ll_name;

    LimelightHelpers.setCameraPose_RobotSpace(ll_name, cameraOffset.getX(), cameraOffset.getY(), cameraOffset.getZ(),
        Units.radiansToDegrees(cameraOffset.getRotation().getX()), Units.radiansToDegrees(cameraOffset.getRotation().getY()), Units.radiansToDegrees(cameraOffset.getRotation().getZ()));
    
        setPipeline(1);
  }

  public LimelightHelpers.PoseEstimate getLLHPoseEstimate(double yaw, double yawRate) {
    LimelightHelpers.SetRobotOrientation(ll_name, yaw, yawRate, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll_name);
  }
  
  public LimelightHelpers.PoseEstimate getLLHPoseEstimateTag1(double yaw, double yawRate) {
    LimelightHelpers.SetRobotOrientation(ll_name, yaw, yawRate, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(ll_name);
  }
  
  public Pose2d getEstimatedPose(double yaw, double yawRate) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

    // Double[] poseArray = { yaw, yawRate, 0., 0., 0., 0. };
    // table.getEntry("robot_orientation_set").setDoubleArray(poseArray);

    Double[] botPoseArray = table.getEntry("botpose_wpiblue").getDoubleArray(new Double[] {});
    if (botPoseArray.length == 0 || botPoseArray[7] == 0) {
      return null;
    }
    Pose2d pose = new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(Units.degreesToRadians(botPoseArray[5])));

    return pose;
  }

  public double getPoseTimestamp() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(ll_name);

    Double[] botPoseArray = table.getEntry("botpose_wpiblue").getDoubleArray(new Double[] {});
    if (botPoseArray.length == 0) {
      return Double.NaN;
    }
    double timeStamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(botPoseArray[6]);

    return timeStamp;
  }

  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(ll_name, index);
  }

  public boolean getObjectedDetected() {
    return LimelightHelpers.getTV(ll_name);
  }

  public int getTagID() {
    return (int) LimelightHelpers.getFiducialID(ll_name);
  }

  public double getAmbiguity() {
    Double[] array = NetworkTableInstance.getDefault().getTable(ll_name).getEntry("rawfiducials")
        .getDoubleArray(new Double[] {});
    if (array.length == 0) {
      return Double.NaN;
    }
    double avg = 0;
    double arrlen = array.length / 7;
    for (int i = 1; i <= arrlen; i++) {
      avg += array[i * 7 - 1] / arrlen;
    }
    return avg;
  }

  public double getTX() {
    return LimelightHelpers.getTX(ll_name);
  }

  public double getTY() {
    return LimelightHelpers.getTY(ll_name);
  }

  public double getTA() {
    return LimelightHelpers.getTA(ll_name);
  }

  public double getCurrentPipeline() {
    return LimelightHelpers.getCurrentPipelineIndex(ll_name);
  }

  public double getDistToCamera() {
    RawFiducial[] fids = LimelightHelpers.getRawFiducials(ll_name);
    if (fids.length == 0) {
      return -1;
    }
    return fids[1].distToCamera;
  }

  public Pose3d getTagPoseRobotSpace() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(ll_name);
  }

  public int getTagCount() {
    try {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(ll_name).tagCount;
    } catch (Exception e) {
      return -1;
    }
  }

  public String getName() {
    return ll_name;
  }

  public enum ReefPosition {
    LEFT, MIDDLE, RIGHT
  }  
}
