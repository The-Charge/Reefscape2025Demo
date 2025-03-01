package frc.robot.commands.vision;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
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
    Rotation2d rot;
    if (tagid == 0) { 
      x = swerve.getClosestTagPose().getX() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.cos(swerve.getClosestTagPose().getRotation().getRadians());
      y = swerve.getClosestTagPose().getY() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.sin(swerve.getClosestTagPose().getRotation().getRadians());
      rot = swerve.getClosestTagPose().getRotation().minus(Rotation2d.k180deg);
    } else {
      x = ApriltagConstants.TAG_POSES[tagid].getX() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.cos(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ());
      y = ApriltagConstants.TAG_POSES[tagid].getY() + ApriltagConstants.CENTER_TO_SCORER_OFFSET * Math.sin(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ());
          rot = ApriltagConstants.TAG_POSES[tagid].getRotation().toRotation2d().minus(Rotation2d.k180deg);
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(x, y, rot)
    );

    PathConstraints constraints = new PathConstraints(SwerveConstants.MAX_SPEED, 3.0, 2 * Math.PI, 4 * Math.PI);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, rot));

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    // run the path

    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag y", y);
    SmartDashboard.putNumber("swerve x", swerve.getPose().getX());
    SmartDashboard.putNumber("swerve y", swerve.getPose().getY());
    intendedPose = new Pose2d(x,y,new Rotation2d(ApriltagConstants.TAG_POSES[tagid].toPose2d().getRotation().getRadians() - Math.PI));
    
    drivetoPose = swerve.driveToPose(intendedPose);
    drivetoPose.schedule();
  }
   
  /*
   * // Create a list of waypoints from poses. Each pose represents one waypoint.
   * // The rotation component of the pose should be the direction of travel. Do
   * not use holonomic rotation.
   * List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
   * new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
   * new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
   * new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
   * );
   * 
   * PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 *
   * Math.PI); // The constraints for this path.
   * // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
   * // You can also use unlimited constraints, only limited by motor torque and
   * nominal battery voltage
   * 
   * // Create the path using the waypoints created above
   * PathPlannerPath path = new PathPlannerPath(
   * waypoints,
   * constraints,
   * null, // The ideal starting state, this is only relevant for pre-planned
   * paths, so can be null for on-the-fly paths.
   * new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can
   * set a holonomic rotation here. If using a differential drivetrain, the
   * rotation will have no effect.
   * );
   * 
   * // Prevent the path from being flipped if the coordinates are already correct
   * path.preventFlipping = true;
   */

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