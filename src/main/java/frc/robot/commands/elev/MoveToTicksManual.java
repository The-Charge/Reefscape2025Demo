package frc.robot.commands.elev;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevConstants;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToTicksManual extends MoveToTicks {
    
    public MoveToTicksManual(ElevSubsystem elevSub) {
        super(elevSub, SmartDashboard.getNumber(ElevConstants.overrideTicksName, 0), false);
    }
}
