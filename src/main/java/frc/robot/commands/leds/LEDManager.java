package frc.robot.commands.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.leds.patterns.LEDDualBreatheAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    private final HeadSubsystem head;
    
    private final LEDDualBreatheAnimation breathe;
    private final LEDPattern hasCoral;
    private final LEDPattern noCoral;
    private final LEDPattern endgame;

    private final double endgameSecs = 20; //seconds left for endgame
    private final double endgameDisplayTime = 1; //amount of time the endgame pattern is displayed for

    private boolean disabledLast;

    public LEDManager(LEDSubsystem ledSub, HeadSubsystem headSub) {
        leds = ledSub;
        head = headSub;
        addRequirements(leds);

        breathe = new LEDDualBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, 0.2, 0.4);
        hasCoral = LEDPattern.solid(LEDConstants.white);
        noCoral = LEDPattern.solid(LEDConstants.orange);
        endgame = LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(3), LEDConstants.ledSpacing);
    }

    @Override
    public void execute() {
        if(DriverStation.isDisabled()) {
            // if(!disabledLast)
            //     breathe.reset();

            breathe.update();
            breathe.evaluate(leds.fullBuff());
            return;
        }

        if(DriverStation.isTeleopEnabled() && Timer.getMatchTime() <= endgameSecs && Timer.getMatchTime() >= endgameSecs - endgameDisplayTime) {
            endgame.applyTo(leds.fullBuff());
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
