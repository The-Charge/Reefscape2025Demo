package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

import java.security.KeyStore.Entry;
import java.util.Map;

import edu.wpi.first.units.Units;

public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED leds;
    private AddressableLEDBuffer buff;
    private LEDPattern effect;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);
        
        leds.setLength(LEDConstants.ledLength);

        // effect = LEDPattern.rainbow(255, 255)
        //     .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(25));
        effect = LEDPattern.steps(Map.of(0, modColor(LEDConstants.chargeGreen), 0.5, modColor(LEDConstants.chargeGold)))
            .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(25));
        effect.applyTo(buff);

        leds.start();
    }

    @Override
    public void periodic() {
        effect.applyTo(buff);
        leds.setData(buff);
    }

    public static Color modColor(Color in) {
        if(!LEDConstants.swapRedGreen)
            return in;
        
        return new Color(in.green, in.red, in.blue);
    }
}
