package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.leds.patterns.LEDScanAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    
    private LEDScanAnimation scan;

    public LEDManager(LEDSubsystem ledSub) {
        leds = ledSub;
        addRequirements(leds);

        scan = new LEDScanAnimation(LEDConstants.chargeGreen, 2);
    }

    @Override
    public void execute() {
        // scan.update();
        // scan.evaluate(leds.segment1());
    }
}
