package frc.robot.commands.head;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HeadConstants;
import frc.robot.subsystems.HeadSubsystem;

public class Indexer extends Command {

    private HeadSubsystem m_head; 

    public Indexer(HeadSubsystem head) {
        this.m_head = head;

        addRequirements(head);
    }

    @Override 
    public void execute() {
        if(m_head.getHeadSensor1())
            m_head.flywheelVBus(HeadConstants.indexerVbus);
    }

    @Override 
    public void end(boolean interrupted) {
        m_head.stop();
    }

    @Override
    public boolean isFinished() {
        return m_head.getHeadSensor2();
    }

}
