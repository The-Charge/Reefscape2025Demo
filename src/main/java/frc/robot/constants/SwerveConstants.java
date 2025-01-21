package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public abstract class SwerveConstants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration.

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Joystick Deadband
    // public static final double LEFT_Y_DEADBAND = 0.1;
    // public static final double LEFT_X_DEADBAND = 0.1;
    // public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final double DEADBAND = 0.1;
    public static final double DRIVE_SPEED = 1; //multiplies translation and rotation, 1 is 100%
}
