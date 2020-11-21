package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

/*
    Controls:
    gamepad1:
    Movement
    Shooting

    gAMEPAD2:
    intake right trigger
    wobblegoal arm move: dpadUp
    wobblegoal grab: triangle

 */
public class TeleOp extends OpMode {
    StandardMechanumDrive drive;
    Actuation actuation;
    GamepadEventPS event1;
    GamepadEventPS event2;

    @Override
    public void init() {
        drive = new StandardMechanumDrive(this.hardwareMap);
        actuation = new Actuation(hardwareMap, drive.getLocalizer());
        event1 = new GamepadEventPS(gamepad1);
        event2 = new GamepadEventPS(gamepad2);
    }

    @Override
    public void loop() {

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );


        if(gamepad2.triangle) actuation.grabWobble();
        else actuation.releaseWobble();

        if(gamepad2.right_trigger > .5) actuation.suck();
        else actuation.stopIntake();

        drive.update();
    }
}
