package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LockOnReef extends InstantCommand {
    private final SwerveSubsystem swerve;
    private Command drivetoPose;
    private VisionSubsystem limelight;
    private Pose2d intendedPose;
    private int tagid;
    public LockOnReef(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid){
        this.swerve = swerve;
        this.limelight = limelight;
        this.tagid = tagid;
        addRequirements(swerve);
        addRequirements(limelight);
    }
 


@Override
  public void initialize(){
     
  }

  @Override 
  public void execute(){
    
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