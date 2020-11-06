package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;

public class TeleOp extends OpMode {
    StandardMechanumDrive drive;
    Actuation actuation;

    @Override
    public void init() {
        drive = new StandardMechanumDrive(this.hardwareMap);
//        actuation = new Actuation(this., drive.getLocalizer());

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

        drive.update();
    }
}
