package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.leds.patterns.LEDStepPattern;
import frc.robot.subsystems.LEDSubsystem;

public abstract class LEDConstants {
    
    public static final int ledPort = 0;
    public static final int ledLength = 44;
    public static final ColorOrder ledColorOrder = ColorOrder.kRGB;
    public static final Distance ledSpacing = Units.Meters.of(1 / 60.0); //60 leds per meter

    public static final LEDSubsystem.Pattern colorTest = new LEDStepPattern(Map.ofEntries(
        Map.entry(0d, new Color("#ff0000")),
        Map.entry(1 / 12.0, new Color("#ff8800")),
        Map.entry(2 / 12.0, new Color("#ffff00")),
        Map.entry(3 / 12.0, new Color("#88ff00")),
        Map.entry(4 / 12.0, new Color("#00ff00")),
        Map.entry(5 / 12.0, new Color("#00ff88")),
        Map.entry(6 / 12.0, new Color("#00ffff")),
        Map.entry(7 / 12.0, new Color("#0088ff")),
        Map.entry(8 / 12.0, new Color("#0000ff")),
        Map.entry(9 / 12.0, new Color("#8800ff")),
        Map.entry(10 / 12.0, new Color("#ff00ff")),
        Map.entry(11 / 12.0, new Color("#ff0088"))
    ), Units.InchesPerSecond.zero());
    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
}
