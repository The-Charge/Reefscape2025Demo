package frc.robot.constants;

import swervelib.parser.PIDFConfig;

public abstract class ClimbConstants {
    
    public static final int motorID = 6;
    public static final double maxVBus = 1;
    public static final double maxCurrent = 40;
    public static final PIDFConfig pidf = new PIDFConfig(0.05, 0.002, 0, 0);
    public static final double minPosTicks = 0;
    public static final double maxPosTicks = Double.MAX_VALUE;
    public static final double tickToDegConversion = 1;
    public static final double targetThresholdDegrees = 1.5;
    public static final double targetThresholdSeconds = 0.1;

    public static final double restingDegrees = 0;
    public static final double activeDegrees = 120;

    public static final String overrideDegName = "Climb Target Override (Deg)";
    public static final String overrideTicksName = "Climb Target Override (Ticks)";
}
