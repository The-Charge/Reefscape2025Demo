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
    public static final int algaeRemLevel = COMPETITION;
    public static final int climbLevel = COMPETITION;
    public static final int elevLevel = COMPETITION;
    public static final int headLevel = COMPETITION;
    public static final int intakeLevel = COMPETITION;
    public static final int ledLevel = COMPETITION;
    public static final SwerveDriveTelemetry.TelemetryVerbosity swerveLevel = TelemetryVerbosity.HIGH;
    public static final int visionLevel = COMPETITION;
    public static final int algaeManipLevel = COMPETITION;
    public static final int robotLevel = COMPETITION; //whole robot telemetry, such as PDP
}
