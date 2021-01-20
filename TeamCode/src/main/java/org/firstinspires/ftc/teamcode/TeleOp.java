package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;
import org.firstinspires.ftc.teamcode.core.gamepad.Toggle;

/*
    Controls:
    Gamepad1:
    Movement
    Shooting

    Gamepad2:
    intake right trigger
    wobblegoal arm move: dpadUp
    wobblegoal grab: triangle

 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    StandardMechanumDrive drive;
    Actuation actuation;
    GamepadEventPS update1;
    GamepadEventPS update2;

    @Override
    public void init() {
        drive = new StandardMechanumDrive(this.hardwareMap);
        Pose2d startPose;
        String serialized = ""; /*hardwareMap.appContext.getSharedPreferences("Auton end pose", Context.MODE_PRIVATE)
                .getString("serialized", "");*/
        if(serialized.equals(""))
            startPose = new Pose2d(0,0,0);
        else startPose = unserialize(serialized);
        drive.setPoseEstimate(startPose);
        actuation = new Actuation(hardwareMap, drive.getLocalizer());
        update1 = new GamepadEventPS(gamepad1);
        update2 = new GamepadEventPS(gamepad2);
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

        if(gamepad2.right_trigger > .5) actuation.suck();
        if(gamepad2.left_trigger > .5) actuation.spitOut();
        if(gamepad2.right_trigger < .5 && gamepad2.left_trigger < .5) actuation.stopIntake();

        /*if(gamepad1.triangle) actuation.wobbleClawClose();
        else actuation.wobbleClawOpen();

        if(gamepad1.square) actuation.wobbleArmUp();
        else actuation.wobbleArmDown();*/

        if(update1.triangle()) {
            if(actuation.isWobbleArmUp())
                actuation.wobbleArmDown();
            else actuation.wobbleArmUp();
        }
        if(update1.square()) {
            if(actuation.isWobbleClawOpen())
                actuation.wobbleClawClose();
            else actuation.wobbleClawOpen();
        }

        if(actuation.hasRings()) {
            actuation.preheatShooter();
            telemetry.addLine("Rings present");
        }
        else actuation.killFlywheel();

       /* if(update1.circle())
            actuation.shoot(drive);
        if (update1.triangle())
            actuation.powerShots(drive);*/

        telemetry.update();
        drive.update();
    }
    static Pose2d unserialize(String s) {
        String[] stringValues = s.substring(1, s.length() - 1).split(",");
        double x = Double.parseDouble(stringValues[0]);
        double y = Double.parseDouble(stringValues[1]);
        double heading = Double.parseDouble(stringValues[2]);
        return new Pose2d(x, y, heading);
    }
}
