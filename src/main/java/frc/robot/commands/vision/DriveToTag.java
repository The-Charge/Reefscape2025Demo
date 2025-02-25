package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToTag extends InstantCommand {
  private final SwerveSubsystem swerve;
  private Command drivetoPose;
  private Pose2d intendedPose;
  private int tagid; 
  public DriveToTag(SwerveSubsystem swerve, int tagid){
      this.swerve = swerve;
      this.tagid = tagid;
      addRequirements(swerve);
  }
 
  @Override
  public void initialize() {
    if (tagid == 0){
      intendedPose = new Pose2d(
      swerve.getClosestTagPose().getX() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.cos(swerve.getClosestTagPose().getRotation().getRadians()),
      swerve.getClosestTagPose().getY() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.sin(swerve.getClosestTagPose().getRotation().getRadians()), 
      new Rotation2d(ApriltagConstants.TAG_POSES[tagid].toPose2d().getRotation().getRadians() - Math.PI)
      );
    }
    else{
      intendedPose = new Pose2d(
      ApriltagConstants.TAG_POSES[tagid].getX() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.cos(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()),
      ApriltagConstants.TAG_POSES[tagid].getY() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.sin(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()), 
      new Rotation2d(ApriltagConstants.TAG_POSES[tagid].toPose2d().getRotation().getRadians() - Math.PI)
      );
    }
    
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