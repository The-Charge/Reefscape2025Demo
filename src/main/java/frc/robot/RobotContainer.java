// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
* little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
* Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
    
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    
    // int rotationXboxAxis = 4;
    
    /*
     * ======================
     * Auto
     * ======================
     */
    private SendableChooser<Command> autoChooser;
    
    /*
     * ======================
     * Swerve
     * ======================
     */
    private final SwerveInputStream swerveInputFieldOriented = SwerveInputStream.of(
        swerve.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX()
    ).withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(SwerveConstants.DEADBAND)
        .scaleTranslation(SwerveConstants.DRIVE_SPEED)
        .scaleRotation(SwerveConstants.DRIVE_SPEED)
        .allianceRelativeControl(true);
    private final SwerveInputStream swereveInputRobotOriented = swerveInputFieldOriented.copy()
        .robotRelative(true)
        .allianceRelativeControl(false);
    private final Command driveFieldOriented = swerve.driveFieldOriented(swerveInputFieldOriented);
    private final Command driveRobotOriented = swerve.driveFieldOriented(swereveInputRobotOriented);
    public boolean isFieldCentric = true;
    
    public RobotContainer() {
        // if (RobotBase.isSimulation()) {
        //     rotationXboxAxis = 2;
        // }
        
        // TeleopDrive teleopDrive = new TeleopDrive(drivebase,
        // () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                               SwerveConstants.LEFT_Y_DEADBAND),
        // () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                               SwerveConstants.LEFT_X_DEADBAND),
        // () -> -driverXbox.getRawAxis(rotationXboxAxis));
        
        swerve.setDefaultCommand(isFieldCentric ? driveFieldOriented : driveRobotOriented);
        
        configureNamedCommands();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driverXbox.b().onTrue((Commands.runOnce(swerve::zeroGyroWithAlliance)));
        driverXbox.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
        driverXbox.back().onTrue(new InstantCommand(() -> {
            isFieldCentric = !isFieldCentric;

            Command curr = swerve.getCurrentCommand();
            if(curr != null)
                curr.cancel();

            swerve.setDefaultCommand(isFieldCentric ? driveFieldOriented : driveRobotOriented);
        }));
        //driverXbox.b().whileTrue(
        //    drivebase.driveToPose(
        //        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
        //                        );
        //driverXbox.start().whileTrue(Commands.none());
        //driverXbox.back().whileTrue(Commands.none());
        //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        //driverXbox.rightBumper().onTrue(Commands.none());
    }
    private void configureNamedCommands() {
        //Pathplanner named commands
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
