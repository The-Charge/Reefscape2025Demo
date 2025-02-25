package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToTag extends Command {
  private final SwerveSubsystem swerve;
  private VisionSubsystem limelight;
  private double sideAdjust;
  private double frontAdjust;
  private ChassisSpeeds alignmentSpeeds;
  private Pose2d tagpose;
  private Rotation2d tagpose_rotation;
  
  private double alignmentSide;
  public AlignToTag(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid, double alignmentSide){
      this.swerve = swerve;
      this.limelight = limelight;
      this.alignmentSide = alignmentSide;
      addRequirements(swerve);
      addRequirements(limelight);
  }

  @Override
  public void initialize(){
    limelight.setPipeline(1);
  }

  @Override 
  public void execute() {
    tagpose = swerve.getClosestReefTagPose();
    tagpose_rotation = tagpose.getRotation().minus(new Rotation2d(Math.PI));
    frontAdjust = MathUtil.clamp((limelight.getFrontAdjustment()), -1, 1);
    sideAdjust = MathUtil.clamp(limelight.getSideAdjustment(alignmentSide), -0.3, 0.3);
    
    SmartDashboard.putNumber("sideval", sideAdjust);
    SmartDashboard.putNumber("frontval", frontAdjust);
    alignmentSpeeds = swerve.getTargetSpeeds(sideAdjust, frontAdjust, tagpose_rotation.getSin(), tagpose_rotation.getCos());
    swerve.drive(alignmentSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}