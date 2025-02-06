package frc.robot.commands.swervedrive.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDrive extends Command {
    
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final BooleanSupplier povUp, povLeft, povDown, povRight;
    private final BooleanSupplier centricToggle;
    private final BooleanSupplier shiftHalf, shiftQuarter;

    private boolean usePOV = false;
    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;
    
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier povUp, BooleanSupplier povLeft, BooleanSupplier povDown, BooleanSupplier povRight, BooleanSupplier centricToggle, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter) {
        this.swerve = swerve;
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
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;
        
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
            .times(SwerveConstants.DRIVE_SPEED) //scale by drive speed percentage
            .times(swerve.isRedAlliance() ? -1 : 1); //switch for red alliance

            
            //translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS), swerve.getSwerveDriveConfiguration());
            //SmartDashboard.putNumber("LimitedTranslation", translation.getX());
            //SmartDashboard.putString("Translation", translation.toString());
            
            // Make the robot move
            if(usePOV) {
                //pov is not effected by drive speed percentage or trigger shifting
                double isRedAlliance = swerve.isRedAlliance() ? -1 : 1;
                ChassisSpeeds povSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX * isRedAlliance, headingY * isRedAlliance);
                swerve.drive(translation, povSpeeds.omegaRadiansPerSecond, isFieldCentric);
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