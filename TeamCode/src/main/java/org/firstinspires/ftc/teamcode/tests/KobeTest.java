package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEventPS;

@TeleOp
public class KobeTest extends OpMode {
    DcMotor shooter;
    GamepadEventPS update;
    double power = 0;
    Servo feeder;
    double feederPosition = 0;

    @Override
    public void init() {
        update = new GamepadEventPS(gamepad1);
        shooter = this.hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder = hardwareMap.servo.get("feeder");

    }

    @Override
    public void loop() {
        if(gamepad1.circle)
            power = 1;

        if(gamepad1.square)
            power = 0;

        if(gamepad1.x) {
            feeder.setPosition(1);
            feeder.setPosition(0);
        }

        if(update.dPadLeft()) {
            if(Math.abs(power) <= 1)
                power -= .01;
        }

        if(update.dPadRight()) {
            if(Math.abs(power) <= 1)
                power += .01;
        }

        if(update.dPadUp())
            feederPosition += .01;
        if(update.dPadDown())
            feederPosition -= .01;

        shooter.setPower(power);
        telemetry.addData("Shooter power", power);
        telemetry.addData("feeder pos", feeder.getPosition());
        telemetry.update();

    }
}
