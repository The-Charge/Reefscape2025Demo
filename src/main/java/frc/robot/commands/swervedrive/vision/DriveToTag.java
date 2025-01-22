// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTag extends InstantCommand {
    private final SwerveSubsystem swerve;
    private Command drivetoPose;
    private VisionSubsystem limelight;
    private Pose2d intendedPose;
    private int tagid;
    public DriveToTag(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid){
        this.swerve = swerve;
        this.limelight = limelight;
        this.tagid = tagid;
        addRequirements(swerve);
        addRequirements(limelight);
        
    }
 


@Override
  public void initialize() {
      intendedPose = new Pose2d(
        ApriltagConstants.TAG_POSES[tagid].getX() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.cos(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()),
        ApriltagConstants.TAG_POSES[tagid].getY() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.sin(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()), 
        new Rotation2d(ApriltagConstants.TAG_POSES[tagid].getZ())
      );
      drivetoPose = swerve.driveToPose(intendedPose);
}

  @Override
  public void execute() {
      drivetoPose.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetoPose != null;
  }

}