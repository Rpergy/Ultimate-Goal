package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadEventPS;
import org.firstinspires.ftc.teamcode.Toggle;

@TeleOp
public class KobeTest extends OpMode {
    DcMotor shooter;
    GamepadEventPS update = new GamepadEventPS(gamepad1);

    double power = 1;

    @Override
    public void init() {
        shooter = this.hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        if(gamepad1.circle)
            power = 1;

        if(gamepad1.square)
            power = 0;

        if(update.dPadLeft()) {
            if(Math.abs(power) <= 1)
                power -= .01;
        }

        if(update.dPadRight()) {
            if(Math.abs(power) <= 1)
                power += .01;
        }

        shooter.setPower(power);
        telemetry.addData("Shooter power:", power);
        telemetry.update();
    }
}
