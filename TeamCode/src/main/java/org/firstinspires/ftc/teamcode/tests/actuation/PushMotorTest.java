package org.firstinspires.ftc.teamcode.tests.actuation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class PushMotorTest extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.square)
            motor.setPower(.7);

        if(gamepad1.circle)
            motor.setPower(-.7);

        if(!gamepad1.circle && !gamepad1.x)
            motor.setPower(0);
    }
}
