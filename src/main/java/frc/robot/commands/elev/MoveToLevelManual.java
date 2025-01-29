package frc.robot.commands.elev;

import frc.robot.subsystems.ElevSubsystem;

public class MoveToLevelManual extends MoveToLevel {
    
    public MoveToLevelManual(ElevSubsystem elevSub) {
        super(elevSub, elevSub.getOverrideLevel(), false);
    }
}
