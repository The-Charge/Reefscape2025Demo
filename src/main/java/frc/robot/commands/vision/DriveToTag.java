package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class DriveToTag extends InstantCommand {
  private final SwerveSubsystem swerve;
  private Command drivetoPose;
  private Pose2d intendedPose;
  private int tagid; 
  private ReefPosition reefPos;

  public DriveToTag(SwerveSubsystem swerve, int tagid, VisionSubsystem.ReefPosition reefPos){
      this.swerve = swerve;
      this.tagid = tagid;
      this.reefPos = reefPos;
      addRequirements(swerve);
  }
 
  @Override
  public void initialize() {
    double offset = 0;
    switch (reefPos) {
      case LEFT:
        offset = ApriltagConstants.LEFT_ALIGN_OFFSET;
        break;
      case RIGHT:
        offset = ApriltagConstants.RIGHT_ALIGN_OFFSET;
        break;
      default:
        break;
    }
    double x, y;
    if (tagid == 0) { 
      x = swerve.getClosestTagPose().getX() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.cos(swerve.getClosestTagPose().getRotation().getRadians());
      y = swerve.getClosestTagPose().getY() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.sin(swerve.getClosestTagPose().getRotation().getRadians());
    } 
    else {
      x = ApriltagConstants.TAG_POSES[tagid].getX() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.cos(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ());
      y = ApriltagConstants.TAG_POSES[tagid].getY() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.sin(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()); 
    }
    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag y", y);
    SmartDashboard.putNumber("swerve x", swerve.getPose().getX());
    SmartDashboard.putNumber("swerve y", swerve.getPose().getY());
    intendedPose = new Pose2d(x,y,new Rotation2d(ApriltagConstants.TAG_POSES[tagid].toPose2d().getRotation().getRadians() - Math.PI));
    
    drivetoPose = swerve.driveToPose(intendedPose);
    drivetoPose.schedule();
  }
   


  @Override 
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    drivetoPose.cancel();
  }

  @Override
  public boolean isFinished() {
    return drivetoPose != null;
  }
}