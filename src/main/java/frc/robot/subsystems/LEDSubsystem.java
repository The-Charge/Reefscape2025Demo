package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED leds;
    private AddressableLEDBuffer buff;
    // private LEDPattern effect;
    private int time = 0;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);
        
        leds.setLength(LEDConstants.ledLength);
        leds.setColorOrder(ColorOrder.kRGB);

        // effect = LEDPattern.rainbow(255, 255)
        //     .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(25));
        // effect = LEDPattern.steps(Map.of(0, modColor(LEDConstants.chargeGreen), 0.5, modColor(LEDConstants.chargeGold)))
        //     .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(25));
        // effect.applyTo(buff);

        leds.start();
    }

    @Override
    public void periodic() {
        // effect.applyTo(buff);

        final double width = 10;
        final double speed = 1;
        final Color col = LEDConstants.chargeGreen;

        double offset = ((Math.sin((Math.PI * time * speed) / (buff.getLength() - width) - 0.5 * Math.PI) + 1) * 0.5) * (buff.getLength() - width);
        for(int i = 0; i < buff.getLength(); i++) {
            double strength = 0;
            if(offset < i && i < width + offset) {
                strength = Math.sin(Math.PI * (i - offset) / width);
            }

            Color pixel = new Color(
                col.red * strength,
                col.green * strength,
                col.blue * strength
            );

            buff.setLED(i, pixel);
        }

        leds.setData(buff);

        time++;
    }
}
