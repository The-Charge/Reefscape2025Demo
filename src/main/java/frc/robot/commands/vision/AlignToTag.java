package frc.robot.commands.vision;

import com.thethriftybot.ThriftyNova.PIDConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class AlignToTag extends Command {
  private final SwerveSubsystem swerve;
  private VisionSubsystem limelight;
  private double sideAdjust;
  private double frontAdjust;
  private ChassisSpeeds alignmentSpeeds;
  private Pose2d tagpose;
  private Rotation2d tagpose_rotation;
  private ReefPosition reefPos;
  private PIDController sideController, frontController;

  private double leftSideAdjust, rightSideAdjust, midSideAdjust; 
  private double tx;

  private double alignmentSide;
  public AlignToTag(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid, double alignmentSide, VisionSubsystem.ReefPosition pos){
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
    sideController.reset();
    frontController.reset();
    tx = limelight.getTX();
    leftSideAdjust = MathUtil.clamp(sideController.calculate(tx + ApriltagConstants.LEFT_ALIGN_OFFSET,0), -0.2, 0.2);
    midSideAdjust = MathUtil.clamp(sideController.calculate(tx,0), -0.2, 0.2);
    rightSideAdjust = MathUtil.clamp(sideController.calculate(tx + ApriltagConstants.RIGHT_ALIGN_OFFSET,0), -0.2, 0.2);
    tagpose = swerve.getClosestReefTagPose();
    tagpose_rotation = tagpose.getRotation().minus(new Rotation2d(Math.PI));

    frontAdjust = MathUtil.clamp(frontController.calculate(limelight.getDistToCamera(), ApriltagConstants.APRILTAG_POSE_OFFSET), -1, 1);
    sideAdjust = MathUtil.clamp(sideController.calculate(tx, 0), -1, 1); // current error = atan((dsin(theta) + k)/(dcos(theta))) - theta; theta = tx, d = getDistToCamera, k = LEFT/RIGHT ALIGN OFFSET. Or use getY instead of TX
    
    SmartDashboard.putNumber("sideval", sideAdjust);
    SmartDashboard.putNumber("frontval", frontAdjust);

    
    alignmentSpeeds = swerve.getTargetSpeeds(sideAdjust, frontAdjust, tagpose_rotation.getSin(), tagpose_rotation.getCos());
    swerve.drive(alignmentSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(1);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
}