// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTag extends InstantCommand {
    // private final SwerveSubsystem swerve;
    private Command drivetoPose;
    // private VisionSubsystem limelight;
    public DriveToTag(SwerveSubsystem swerve, VisionSubsystem limelight, double tag, boolean ManualControl){
        // this.swerve = swerve;
        // this.limelight = limelight;
        addRequirements(swerve);
        addRequirements(limelight);
    }
 


@Override
  public void initialize() {
      
    
}

  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetoPose != null && drivetoPose.isFinished();
  }

}