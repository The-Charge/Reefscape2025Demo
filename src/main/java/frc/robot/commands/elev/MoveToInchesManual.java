package frc.robot.commands.elev;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevConstants;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToInchesManual extends MoveToInches {
    
    public MoveToInchesManual(ElevSubsystem elevSub) {
        super(elevSub, SmartDashboard.getNumber(ElevConstants.overrideInName, 0), false);
    }
}
