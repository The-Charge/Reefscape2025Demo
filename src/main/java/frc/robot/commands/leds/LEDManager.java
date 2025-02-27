package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.leds.patterns.LEDBreatheAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    private final HeadSubsystem head;
    
    private final LEDBreatheAnimation breathe;
    private final LEDPattern hasCoral;
    private final LEDPattern noCoral;

    private boolean disabledLast;

    public LEDManager(LEDSubsystem ledSub, HeadSubsystem headSub) {
        leds = ledSub;
        head = headSub;
        addRequirements(leds);

        breathe = new LEDBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, 0.2, 0.4);
        hasCoral = LEDPattern.solid(LEDConstants.white);
        noCoral = LEDPattern.solid(LEDConstants.orange);
    }

    @Override
    public void execute() {
        if(DriverStation.isDisabled()) {
            if(!disabledLast)
                breathe.reset();

            breathe.update();
            breathe.evaluate(leds.fullBuff());
            return;
        }

        if(head.getHasCoral()) {
            hasCoral.applyTo(leds.fullBuff());
            return;
        }

        noCoral.applyTo(leds.fullBuff());

        disabledLast = DriverStation.isDisabled();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
