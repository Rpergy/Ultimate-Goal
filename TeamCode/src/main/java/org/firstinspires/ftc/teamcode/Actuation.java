package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.nio.charset.CharacterCodingException;

public class Actuation {
    static Vector2d redGoal = new Vector2d(72, -44);
    private double WOBBLE_GRAB = 0; //TODO: Find these vals
    private double WOBBLE_RELEASE = 1;
    DcMotor shoot, intake, wobbleLift;
    Servo turn, wobbleGrab, wobbleTurn;
    LinearOpMode opMode;
    Localizer localizer;

    Actuation(LinearOpMode linearOpMode, Localizer localizer) {
        opMode = linearOpMode;
        this.localizer = localizer;

        intake = linearOpMode.hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoot = linearOpMode.hardwareMap.dcMotor.get("shooter");
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turn = linearOpMode.hardwareMap.servo.get("shootTurn");
        turn.setPosition(0);

        wobbleGrab = linearOpMode.hardwareMap.servo.get("wobbleGrab");
        wobbleGrab.setPosition(WOBBLE_GRAB); //TODO: Find "grab" pos
    }

    void suck() {
        intake.setPower(1);
    }
    void stopIntake() {
        intake.setPower(0);
    }
    void spitOut() {
        intake.setPower(-1);
    }

    //TODO: Different shoot powers for different distances?
    void shoot() {
        Pose2d pose = localizer.getPoseEstimate();
        double bearing = redGoal.angleBetween(pose.vec()) + pose.getHeading(); // should be plus or minus?
        turn.setPosition(bearing); //TODO: Figure out how to translate radians to turn into turn pos for servo

        shoot.setPower(1);
        opMode.sleep(2000);
        shoot.setPower(0);
    }

    void shoot(Vector2d target) {
        Pose2d pose = localizer.getPoseEstimate();
        double bearing = target.angleBetween(pose.vec()) + pose.getHeading(); // should be plus or minus?
        turn.setPosition(bearing); //TODO: Figure out how to translate radians to turn into turn pos for servo

        shoot.setPower(1);
        opMode.sleep(2000);
        shoot.setPower(0);
    }

    void grabWobble() {
        wobbleGrab.setPosition(WOBBLE_GRAB);
    }

    void releaseWobble() {
        wobbleGrab.setPosition(WOBBLE_RELEASE);
    }

}
