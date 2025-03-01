package frc.robot.commands.algaemanip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeManipConstants;
import frc.robot.subsystems.AlgaeManipSubsystem;

public class AlgaeManipSpin extends Command {
    
    private final AlgaeManipSubsystem manip;
    private final boolean in;

    public AlgaeManipSpin(AlgaeManipSubsystem algaeManipSub, boolean spinIn) {
        manip = algaeManipSub;
        in = spinIn;

        addRequirements(manip);
    }

    @Override
    public void initialize() {
        manip.rollerVBus((in ? 1 : -1) * AlgaeManipConstants.spinVbus);
    }
    @Override
    public void end(boolean interrupted) {
        manip.rollerStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
