package org.firstinspires.ftc.teamcode.tests;

import android.view.KeyEvent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEventPS;

@TeleOp
public class ContinuousServoTest extends OpMode {
    CRServo servo;
    double power = 0;
    GamepadEventPS update;

    @Override
    public void init() {
        servo = hardwareMap.crservo.get("servo");
        update = new GamepadEventPS(this.gamepad1);

    }

    @Override
    public void loop() {
//        if
        if(gamepad1.circle)
            power = -.2;
        if(gamepad1.square)
            power = -.33;
        if(gamepad1.triangle)
            power = -.46;

        if (update.dPadLeft())
            power -= .01;
        if (update.dPadRight())
            power += .01;
        servo.setPower(power);
        telemetry.addData("power", power);
    }
}
