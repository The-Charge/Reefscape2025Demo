package frc.robot.constants;

import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public abstract class TelemetryConstants {

    public static final int COMPETITION = 0; //only what drivers need to see
    public static final int LOW = 1;
    public static final int MEDIUM = 2;
    public static final int HIGH = 3;
    public static final int EYE_OF_SAURON = 4; //absolutely everything
    
    /*
     * Please set everything to the lowest you can during competition to reduce load on rio and radio
     */
    public static final int algaeRemLevel = HIGH;
    public static final int climbLevel = HIGH;
    public static final int elevLevel = HIGH;
    public static final int headLevel = HIGH;
    public static final int intakeLevel = HIGH;
    public static final int ledLevel = HIGH;
    public static final SwerveDriveTelemetry.TelemetryVerbosity swerveLevel = TelemetryVerbosity.HIGH;
    public static final int visionLevel = HIGH;
    public static final int robotLevel = HIGH; //whole robot telemetry, such as PDP
}
