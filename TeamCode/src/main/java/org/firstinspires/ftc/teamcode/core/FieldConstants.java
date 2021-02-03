package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static java.lang.Math.toRadians;

@Config
public class FieldConstants {
    public static double MAX_EDGE = 72;
    public static double SHOOT_LINE = 0; // Theoretical: 12

    // For shooting coordinates
    public static Vector2d redGoal = new Vector2d(72, -44);
    public static Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    public static Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    public static Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);

    // For path coordinates
    public static Pose2d centerA = new Pose2d(12,-60, toRadians(90));
    public static Pose2d centerB = new Pose2d(36,-32, toRadians(90));
    public static Pose2d centerC = new Pose2d(70, -65, toRadians(90)); // theoretical: x = 60, y = -60
    public static Pose2d backPose = new Pose2d(-55, -30, toRadians(0)); //theoretical: x = -31, y = -48
    public static Pose2d startPose = new Pose2d(-63, -34);
}
