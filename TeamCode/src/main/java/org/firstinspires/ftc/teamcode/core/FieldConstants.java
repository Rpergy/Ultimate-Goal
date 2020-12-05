package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class FieldConstants {
    public static final double MAX_EDGE = 72;
    public static final Vector2d redGoal = new Vector2d(72, -44);
    public static final Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    public static final Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    public static final Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);
    public static final Vector2d centerA = new Vector2d(12,-60);
    public static final Vector2d centerB = new Vector2d(36,-36);
    public static final Vector2d centerC = new Vector2d(60, -60);
    public static final Pose2d startPose = new Pose2d(-63, -36);
}
