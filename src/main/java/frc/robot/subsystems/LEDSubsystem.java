package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import edu.wpi.first.units.Units;

public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED leds;
    private AddressableLEDBuffer buff;
    private LEDPattern rainbow;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);
        
        leds.setLength(LEDConstants.ledLength);

        // buff.setRGB(1, 255, 0, 0);
        
        rainbow = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(10));
        rainbow.applyTo(buff);

        leds.start();
    }

    @Override
    public void periodic() {
        rainbow.applyTo(buff);
        leds.setData(buff);
    }
}
