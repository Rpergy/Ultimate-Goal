package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;

import java.nio.charset.CharacterCodingException;

public class Actuation {
    static Vector2d redGoal = new Vector2d(72, -44);
    private final double WOBBLE_GRAB = 0; //TODO: Find these vals
    private final double WOBBLE_RELEASE = 1;
    private final double RESTING_TURNING_POS = 0;
    private final int TURN_OFFSET = 150;
    DcMotor shoot, intake, wobbleLift;
    Servo turn, wobbleGrab, wobbleTurn;
    LinearOpMode opMode;
    Localizer localizer;

    Actuation(LinearOpMode linearOpMode, Localizer localizer) {
        opMode = linearOpMode;
        this.localizer = localizer;

        if (linearOpMode.hardwareMap.dcMotor.contains("intake")) {
            intake = linearOpMode.hardwareMap.dcMotor.get("intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (linearOpMode.hardwareMap.dcMotor.contains("shooter")) {
            shoot = linearOpMode.hardwareMap.dcMotor.get("shooter");
            shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (linearOpMode.hardwareMap.servo.contains("shootTurn")) {
            turn = linearOpMode.hardwareMap.servo.get("shootTurn");
            turn.setPosition(RESTING_TURNING_POS);
            turn.scaleRange(0, 300);
        }

        if (linearOpMode.hardwareMap.servo.contains("wobbleGrab")) {
            wobbleGrab = linearOpMode.hardwareMap.servo.get("wobbleGrab");
            wobbleGrab.setPosition(WOBBLE_GRAB); //TODO: Find "grab" pos
        }
    }

    void suck() {
        if (intake != null)
            intake.setPower(1);
    }

    void stopIntake() {
        if (intake != null)
            intake.setPower(0);
    }

    void spitOut() {
        if (intake != null)
            intake.setPower(-1);
    }

    // TODO: Different shoot powers for different distances?
    void shoot() {
        Pose2d pose = localizer.getPoseEstimate();
        double bearing = redGoal.angleBetween(pose.vec()) + pose.getHeading(); // should be plus or minus?
        turn.setPosition(bearing); // TODO: Figure out how to translate radians to turn into turn pos for servo

        shoot.setPower(1);
        opMode.sleep(2000);
        shoot.setPower(0);
    }

    void shoot(Vector2d target) {
        if (shoot != null && turn != null) {
            Pose2d pose = localizer.getPoseEstimate();
            double bearing = target.angleBetween(pose.vec()) + pose.getHeading(); // should be plus or minus?
            if (Math.abs(bearing) > 150)
                turn.setPosition(bearing); // TODO: Figure out how to translate radians to turn into turn pos for servo

            shoot.setPower(1);
            opMode.sleep(2000);
            shoot.setPower(0);
        }
    }

    void grabWobble() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_GRAB);
    }

    void releaseWobble() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_RELEASE);
    }

    /**
     * From -150 (all the way left) to 150 (all the way right) degrees.
     */
    void turnShooter(double angle) {
        double maxBearing = 300;
        turn.setPosition(angle + TURN_OFFSET);
    }


}
