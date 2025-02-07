package frc.robot.commands.swervedrive.drivebase;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class POVDrive extends Command {
    private final SwerveSubsystem swerve;
    private final BooleanSupplier povDown, povDownleft, povDownRight, povLeft, povRight, povUp, povUpLeft, povUpRight;

    public POVDrive(SwerveSubsystem swerve, BooleanSupplier povDown, BooleanSupplier povDownleft, BooleanSupplier povDownRight, BooleanSupplier povLeft, BooleanSupplier povRight, BooleanSupplier povUp, BooleanSupplier povUpLeft,BooleanSupplier povUpRight) {
        this.swerve = swerve;
        this.povDown = povDown;
        this.povDownleft = povDownleft;
        this.povDownRight = povDownRight;
        this.povLeft = povLeft;
        this.povRight = povRight;
        this.povUp = povUp;
        this.povUpLeft = povUpLeft;
        this.povUpRight = povUpRight;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;

        if (povDown.getAsBoolean()) {
            headingY = -1;
        } else if (povDownleft.getAsBoolean()) {
            headingX = 1;
            headingY = -1;
        } else if (povDownRight.getAsBoolean()) {
            headingX = -1;
            headingY = -1;
        } else if (povLeft.getAsBoolean()) {
            headingX = 1;
        } else if (povRight.getAsBoolean()) {
            headingX = -1;
        } else if (povUp.getAsBoolean()) {
            headingY = 1;
        } else if (povUpLeft.getAsBoolean()) {
            headingX = 1;
            headingY = 1;
        } else if (povUpRight.getAsBoolean()) {
            headingX = -1;
            headingY = 1;
        }

        double isRedAlliance = swerve.isRedAlliance() ? -1 : 1;
        ChassisSpeeds povSpeeds = swerve.getTargetSpeeds(0, 0, headingX * isRedAlliance, headingY * isRedAlliance);
        Translation2d translation = new Translation2d(0, 0); // No translation

        swerve.drive(translation, povSpeeds.omegaRadiansPerSecond, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}