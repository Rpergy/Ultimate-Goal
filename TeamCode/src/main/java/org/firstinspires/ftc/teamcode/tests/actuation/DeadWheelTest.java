package org.firstinspires.ftc.teamcode.tests.actuation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DeadWheelTest extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("frontLeft (left)", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight (right)", frontRight.getCurrentPosition());
        telemetry.addData("backRight (sideways)", backRight.getCurrentPosition());
        telemetry.update();
    }
}
