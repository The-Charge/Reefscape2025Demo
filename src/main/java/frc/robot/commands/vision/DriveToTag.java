package frc.robot.commands.vision;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class DriveToTag extends Command {
  private final SwerveSubsystem swerve;
  private Command drivetoPose;
  private Pose2d intendedPose;
  private int tagid; 
  private ReefPosition reefPos;
  private DoubleSupplier vX, vY, heading;
  private BooleanSupplier buttonY;

  public DriveToTag(SwerveSubsystem swerve, int tagid, VisionSubsystem.ReefPosition reefPos, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier buttonY){
      this.swerve = swerve;
      this.tagid = tagid;
      this.reefPos = reefPos;
      this.vX = vX;
      this.vY = vY;
      this.heading = heading;
      this.buttonY = buttonY;

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
    Rotation2d rot2d;
    if (tagid == 0) { 
      rot2d = swerve.getClosestTagPose().getRotation();
      x = swerve.getClosestTagPose().getX() + (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset) * Math.cos(rot2d.getRadians());
      y = swerve.getClosestTagPose().getY() + (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset) * Math.sin(rot2d.getRadians());
    } else {
      rot2d = ApriltagConstants.TAG_POSES[tagid].getRotation().toRotation2d();
      x = ApriltagConstants.TAG_POSES[tagid].getX() + (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset) * Math.cos(rot2d.getRadians());
      y = ApriltagConstants.TAG_POSES[tagid].getY() + (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset) * Math.sin(rot2d.getRadians()); 
    }
    rot2d = rot2d.minus(Rotation2d.k180deg);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      swerve.getPose(),
      new Pose2d(x, y, rot2d)
    );

    PathConstraints constraints = new PathConstraints(SwerveConstants.MAX_SPEED, 3.0, 2 * Math.PI, 4 * Math.PI);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, rot2d));

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag y", y);
    SmartDashboard.putNumber("swerve x", swerve.getPose().getX());
    SmartDashboard.putNumber("swerve y", swerve.getPose().getY());

    // run the path
    drivetoPose = AutoBuilder.followPath(path).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

  }
   
  @Override 
  public void execute() {
    // System.out.println("in execute");
    drivetoPose.schedule();
    // if (drivetoPose != null) drivetoPose.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    // if (interrupted) System.out.println("in cancel...");
    // else System.out.println("interrupted false");
    
    if (drivetoPose != null) drivetoPose.cancel();
    // swerve.drive(Translation2d.kZero, 0, false);
  }

  @Override
  public boolean isFinished() {
    // System.out.println("in isFinished");
    
    // if (drivetoPose != null && drivetoPose.isFinished()) {
    //   System.out.println("im done");
    // } else {
    //   System.out.println("not done");
    // }
    // boolean isf = vX.getAsDouble() != 0 || vY.getAsDouble() != 0 || heading.getAsDouble() != 0;
    // System.out.println("ISF ISF ISF ISF ISF ISF\n\n\n" + isf + "\n" + isf + "\n" + isf + "\n" + isf +"\n" + isf +"\n" + isf +"\n" + isf +"\n" + isf + "\n\n");
    return false;
    // return isf;
  }
}