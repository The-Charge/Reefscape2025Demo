package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED leds;
    private AddressableLEDBuffer buff;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);
        
        leds.setLength(LEDConstants.ledLength);

        buff.setRGB(1, 255, 0, 0);

        leds.setData(buff);
        leds.start();
    }

    @Override
    public void periodic() {
        
    }
}
