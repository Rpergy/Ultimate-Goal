package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

@TeleOp
public class KobeTest extends OpMode {
    DcMotor shooter;
    GamepadEventPS update;
    double power = 0;
    Servo feeder;
    double feederPosition = 0;
    final double initial = .34;
    final double launch = .38;
    VoltageSensor voltage;

    @Override
    public void init() {
        update = new GamepadEventPS(gamepad1);
        shooter = this.hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder = hardwareMap.servo.get("feeder");
        voltage = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        if(update.rightBumper())
            feeder.setPosition(launch);

        if(update.leftBumper())
            feeder.setPosition(initial);

        if(gamepad1.square)
            power = 1;

        if(gamepad1.circle)
            power = 0;

        if(update.dPadLeft()) {
            if(Math.abs(power) >= 0.01)
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
        feeder.setPosition(feederPosition);
        telemetry.addData("Shooter power", power);
        telemetry.addData("square to put in feed pos", "triangle to put in initial pos");
        telemetry.addData("feeder pos", feeder.getPosition());
        telemetry.addData("voltage", voltage.getVoltage());
        telemetry.update();

    }
}
