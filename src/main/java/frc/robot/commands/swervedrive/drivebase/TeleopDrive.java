package frc.robot.commands.swervedrive.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDrive extends Command {
    
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final BooleanSupplier centricToggle;
    private final BooleanSupplier shiftHalf, shiftQuarter;

    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;
    
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading,
                       BooleanSupplier centricToggle, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        this.centricToggle = centricToggle;
        this.shiftHalf = shiftHalf;
        this.shiftQuarter = shiftQuarter;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        // Handle field-centric toggle
        if(centricToggle.getAsBoolean() && !centricToggleLast) {
            isFieldCentric = !isFieldCentric;
        }
        centricToggleLast = centricToggle.getAsBoolean();

        // Calculate speed multiplier
        double shiftScalar = 1;
        if(shiftQuarter.getAsBoolean())
            shiftScalar = 0.25;
        else if(shiftHalf.getAsBoolean())
            shiftScalar = 0.5;

        // Calculate rotation
        double rotationSpeed = heading.getAsDouble() * swerve.getSwerveController().config.maxAngularVelocity;
        rotationSpeed *= shiftScalar * SwerveConstants.DRIVE_SPEED;

        // Calculate translation
        Translation2d translation = new Translation2d(vX.getAsDouble(), vY.getAsDouble())
            .times(SwerveConstants.MAX_SPEED)
            .times(shiftScalar)
            .times(SwerveConstants.DRIVE_SPEED)
            .times(isFieldCentric ? swerve.isRedAlliance() ? -1 : 1 : 1);

        // Drive normally
        swerve.drive(translation, rotationSpeed, isFieldCentric);
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