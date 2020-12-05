package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.*;

public class ActuationConstants {

    static Vector2d redGoal = new Vector2d(72, -44);
    static Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    static Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    static Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);

    static final double FLYWHEEL_RADIUS = toMeters(2);
    static final double LAUNCHER_ANGLE = toRadians(20);
    static final double LAUNCHER_HEIGHT = toMeters(14); // Measure where center of mass of disk would be before being launched.
    //TODO: Get a better measurement of power shot heights, considering the slant.
    static final double POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT = toMeters(23.5) - LAUNCHER_HEIGHT;
    static final double TOWER_GOAL_VERTICAL_DISPLACEMENT = toMeters(35.5) - LAUNCHER_HEIGHT;

    // Wobble Grabber Constants
    //TODO: Measure Wobble Constants.
    static final double WOBBLE_GRAB = 0;
    static final double WOBBLE_RELEASE = 1;

    static final double WOBBLE_ARM_UP = 1;
    static final double WOBBLE_ARM_DOWN = 0;
    static final double RESTING_TURNING_POS = 0;

    public enum Target {
        POWER_SHOT_LEFT,
        POWER_SHOT_MIDDLE,
        POWER_SHOT_RIGHT,
        TOWER_GOAL;

        Vector2d pos() {
            switch (this) {
                case POWER_SHOT_LEFT:
                    return leftPowerShot;
                case POWER_SHOT_MIDDLE:
                    return centerPowerShot;
                case POWER_SHOT_RIGHT:
                    return rightPowerShot;
                case TOWER_GOAL:
                    return redGoal;
                default:
                    return new Vector2d();
            }
        }
    }

    /**
     * Converts inches to meters
     * @param in Length, in inches
     * @return Length, in meters
     */
    static double toMeters(double in) { return in / 39.3700787; }
}
