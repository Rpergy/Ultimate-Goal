package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static java.lang.Math.toRadians;

public class FieldConstants {
    public static final double MAX_EDGE = 72;
    public static final double SHOOT_LINE = 12;

    // For shooting coordinates
    public static final Vector2d redGoal = new Vector2d(72, -44);
    public static final Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    public static final Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    public static final Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);

    // For path coordinates
    public static final Pose2d centerA = new Pose2d(12,-60, toRadians(90));
    public static final Pose2d centerB = new Pose2d(36,-36, toRadians(90));
    public static final Pose2d centerC = new Pose2d(65, -60, toRadians(90));
    public static final Pose2d backPose = new Pose2d(-55, -30, toRadians(0)); //theoretical: x = -31, y = -48
    public static final Pose2d startPose = new Pose2d(-63, -36);
}
