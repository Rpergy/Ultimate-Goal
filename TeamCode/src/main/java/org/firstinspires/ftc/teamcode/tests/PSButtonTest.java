package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

@TeleOp
public class PSButtonTest extends OpMode {
    GamepadEventPS update;
    boolean squarePressed = false;
    boolean circlePressed = false;
    boolean trianglePressed = false;
    boolean xPressed = false;

    @Override
    public void init() {
        update = new GamepadEventPS(gamepad1);
    }

    @Override
    public void loop() {
        squarePressed = false;
        circlePressed = false;
        trianglePressed = false;
        xPressed = false;

        if(gamepad1.square) squarePressed = true;
        if(gamepad1.circle) circlePressed = true;
        if(gamepad1.triangle) trianglePressed = true;
        if(gamepad1.x) xPressed = true;

        telemetry.addData("Square", squarePressed);
        telemetry.addData("Circle", circlePressed);
        telemetry.addData("Triangle", trianglePressed);
        telemetry.addData("X", xPressed);
        telemetry.update();
    }
}
