package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.*;

import org.dyn4j.geometry.Rotation;

public class TeleopDrive extends Command {
    
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;

    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final BooleanSupplier povUp, povLeft, povDown, povRight;
    private final BooleanSupplier centricToggle;
    private final BooleanSupplier shiftHalf, shiftQuarter;
    private final BooleanSupplier reefLock;

    private boolean usePOV = false;
    
    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;
    
    public TeleopDrive(SwerveSubsystem swerve, VisionSubsystem limelight, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier povUp, BooleanSupplier povLeft, BooleanSupplier povDown, BooleanSupplier povRight, BooleanSupplier centricToggle, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter, BooleanSupplier useReefLock) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        this.povUp = povUp;
        this.povLeft = povLeft;
        this.povDown = povDown;
        this.povRight = povRight;
        this.centricToggle = centricToggle;
        this.shiftHalf = shiftHalf;
        this.shiftQuarter = shiftQuarter;
        reefLock = useReefLock;
        addRequirements(limelight);
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;
        double tagYaw = 0;
    
        if(povUp.getAsBoolean())  {
            usePOV = true;
            headingY = 1;
        }
        else if(povLeft.getAsBoolean()) {
            usePOV = true;
            headingX = 1;
        }
        else if(povDown.getAsBoolean()) {
            usePOV = true;
            headingY = -1;
        }
        else if(povRight.getAsBoolean()) {
            usePOV = true;
            headingX = -1;
        }

        if(centricToggle.getAsBoolean() && !centricToggleLast) {
            isFieldCentric = !isFieldCentric;
        }
        centricToggleLast = centricToggle.getAsBoolean();

        double shiftScalar = 1;
        if(shiftQuarter.getAsBoolean())
            shiftScalar = 0.25;
        else if(shiftHalf.getAsBoolean())
            shiftScalar = 0.5;
        
        double rotationSpeed = 0;
        if(heading.getAsDouble() != 0) {
            rotationSpeed = heading.getAsDouble() * swerve.getSwerveController().config.maxAngularVelocity;
            usePOV = false;

            rotationSpeed *= shiftScalar;
            rotationSpeed *= SwerveConstants.DRIVE_SPEED;
        }
        
        Translation2d translation = new Translation2d(vX.getAsDouble(), vY.getAsDouble())
            .times(SwerveConstants.MAX_SPEED) //Limit velocity
            .times(shiftScalar) //trigger shifting scalar
            .times(SwerveConstants.DRIVE_SPEED); //scale by drive speed percentage


        //pov is not effected by drive speed percentage or trigger shifting
        ChassisSpeeds povSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

        //translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS), swerve.getSwerveDriveConfiguration());
        //SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        //SmartDashboard.putString("Translation", translation.toString());
        
        // Make the robot move
        if(usePOV) {
            swerve.drive(translation, povSpeeds.omegaRadiansPerSecond, isFieldCentric);
        } 
        else if (reefLock.getAsBoolean()) {
            List<Pose2d> tagposes = new ArrayList<Pose2d>();
            ChassisSpeeds reefLockSpeeds = new ChassisSpeeds();

            for (int i = 1; i <= 22; i++) {
                if (i == 4 || i == 5 || i == 14 || i == 15 || i == 3 || i == 16)
                    continue;
                tagposes.add(ApriltagConstants.TAG_POSES[i].toPose2d());
            }

            reefLockSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), swerve.getPose().nearest(tagposes).getRotation().minus(new Rotation2d(Math.PI)).getSin(), swerve.getPose().nearest(tagposes).getRotation().minus(new Rotation2d(Math.PI)).getCos());

            SmartDashboard.putNumber("Swerve Heading", swerve.getHeading().getRadians());
            SmartDashboard.putNumber("omegaRadiansPerSecond", reefLockSpeeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber("CLosest Tag", swerve.getPose().nearest(tagposes).getRotation().getRadians());
            swerve.drive(translation, reefLockSpeeds.omegaRadiansPerSecond, isFieldCentric);
        } 
        else {
            swerve.drive(translation, rotationSpeed, isFieldCentric);
        }

    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}