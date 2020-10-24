package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEventPS;
import org.firstinspires.ftc.teamcode.ValueCycler;

@TeleOp
public class BasicServoTest extends OpMode {
    Servo servo;
    GamepadEventPS update;
    double position = 0;
    double increment = .1;
    /*ValueCycler positionIncrements = new ValueCycler(gamepad1,
            new double[]{.01, .05, .1, .2, .3},
            ValueCycler.CONTROL_PAIRS.D_PAD_VERTICAL, update);*/
    int index = 0;

    @Override
    public void init() {

        servo = hardwareMap.servo.get("servo");
        update = new GamepadEventPS(gamepad1);
        
    }

    @Override
    public void loop() {

//        increment = positionIncrements.update();

        if (update.circle())
            servo.setPosition(0);
        if (update.x())
            servo.setPosition(1);

        if (update.dPadLeft())
            position -= increment;
        if (update.dPadRight())
            position += increment;

        servo.setPosition(position);
        telemetry.addData("Increment level", increment);
        telemetry.addData("Press a and b to move to either ends", "");
        telemetry.addData("Current position", servo.getPosition());
    }
}
