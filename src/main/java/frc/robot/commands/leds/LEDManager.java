package frc.robot.commands.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.leds.patterns.LEDDualBreatheAnimation;
import frc.robot.commands.leds.patterns.LEDScanAnimation;
import frc.robot.commands.leds.patterns.LEDStepsAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    private final HeadSubsystem head;
    private final ElevSubsystem elev;
    private final CommandXboxController driver1, driver2;
    
    private final LEDDualBreatheAnimation breathe;
    private final LEDStepsAnimation steps;
    private final LEDScanAnimation scan;
    // private final LEDPattern rainbow;
    private final LEDPattern hasCoral;
    private final LEDPattern noCoral;
    private final LEDPattern endgame;
    private final LEDPattern branchAligned;

    private boolean endgameStarted;
    private int ledMode;
    private final int ledModeCount = 5;

    public LEDManager(LEDSubsystem ledSub, HeadSubsystem headSub, ElevSubsystem elevSub, CommandXboxController driver1, CommandXboxController driver2) {
        leds = ledSub;
        head = headSub;
        elev = elevSub;
        this.driver1 = driver1;
        this.driver2 = driver2;
        addRequirements(leds);

        endgameStarted = false;
        ledMode = 0;

        breathe = new LEDDualBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, 0.2, 0.4);
        steps = new LEDStepsAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, Units.MetersPerSecond.of(2));
        scan = new LEDScanAnimation(LEDConstants.chargeGold, 16, 4);
        // rainbow = LEDPattern.rainbow(255, 255)
        //     .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(4), LEDConstants.ledSpacing);

        hasCoral = LEDPattern.solid(LEDConstants.white);
        noCoral = LEDPattern.solid(LEDConstants.orange);
        endgame = LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(3), LEDConstants.ledSpacing);
        branchAligned = LEDPattern.solid(LEDConstants.chargeGreen).blink(Units.Seconds.of(0.33), Units.Seconds.of(0.33));
    }

    public void incrementMode() {
        ledMode = (ledMode + 1) % ledModeCount;
    }

    @Override
    public void execute() {
        if(DriverStation.isDisabled()) {
            breathe.update();
            breathe.evaluate(leds.fullBuff());
            // steps.update();
            // steps.evaluate(leds.fullBuff());
            // scan.update();
            // scan.evaluate(leds.fullBuff());
            // rainbow.applyTo(leds.fullBuff());
            return;
        }

        String ledName;
        switch(ledMode) {
            default:
            case 0: {
                ledName = "Default";
                if(head.getHasCoral()) {
                    if(LEDConstants.branchAlignLevels.contains(elev.getPositionLevel()) && head.getBranchSensor(elev.getPositionLevel())) {
                        branchAligned.applyTo(leds.fullBuff());
                        return;
                    }
        
                    hasCoral.applyTo(leds.fullBuff());
                    return;
                }
        
                noCoral.applyTo(leds.fullBuff());
                break;
            }
            case 1: {
                ledName = "Endgame";
                endgame.applyTo(leds.fullBuff());
                break;
            }
            case 2: {
                ledName = "Breathe";
                breathe.evaluate(leds.fullBuff());
                breathe.update();
                break;
            }
            case 3: {
                ledName = "Steps";
                steps.evaluate(leds.fullBuff());
                steps.update();
                break;
            }
            case 4: {
                ledName = "Scan";
                scan.evaluate(leds.fullBuff());
                scan.update();
                break;
            }
        }
        SmartDashboard.putString("LED Mode", ledName);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    //called in disabled init
    public void resetEndgameStarted() {
        endgameStarted = false;
    }
}
