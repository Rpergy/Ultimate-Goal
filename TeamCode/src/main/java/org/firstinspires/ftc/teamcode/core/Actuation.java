package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.*;

public class Actuation {

    DcMotor shoot, intake;
    Servo turn, wobbleGrab, wobbleArm;
    HardwareMap hardwareMap;
    Localizer localizer;
    VoltageSensor voltage;
    LinearOpMode linearOpMode;

    /**
     * For Autonomous initialization specifically.
     * @param linearOpMode
     * @param localizer
     */
    public Actuation(LinearOpMode linearOpMode, Localizer localizer) {
        this(linearOpMode.hardwareMap, localizer);
        this.linearOpMode = linearOpMode;
    }

    /**
     * For TeleOp initialization specifically.
     * @param hardwareMap
     * @param localizer
     */
    public Actuation(HardwareMap hardwareMap, Localizer localizer) {
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;

        voltage = hardwareMap.voltageSensor.iterator().next();

        if (hardwareMap.dcMotor.contains("intake")) {
            intake = hardwareMap.dcMotor.get("intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (hardwareMap.dcMotor.contains("shooter")) {
            shoot = hardwareMap.dcMotor.get("shooter");
            shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (hardwareMap.servo.contains("shootTurn")) {
            turn = hardwareMap.servo.get("shootTurn");
            turn.setPosition(RESTING_TURNING_POS);
            turn.scaleRange(0, 300);
        }

        if (hardwareMap.servo.contains("wobbleGrab")) {
            wobbleGrab = hardwareMap.servo.get("wobbleGrab");
            wobbleGrab.setPosition(WOBBLE_GRAB); //TODO: Find "grab" pos
        }

        if(hardwareMap.servo.contains("wobbleArm")) {
            wobbleArm = hardwareMap.servo.get("wobbleArm");
            wobbleArm.setPosition(WOBBLE_ARM_UP);
        }
    }

    public void suck() {
        if (intake != null)
            intake.setPower(1);
    }

    public void stopIntake() {
        if (intake != null)
            intake.setPower(0);
    }

    void spitOut() {
        if (intake != null)
            intake.setPower(-1);
    }

    /**
     * Turns the shooter (or robot if necessary) to face the red goal, then fires.
     */
    public void shoot(StandardMechanumDrive drive) {
        shoot(redGoal, drive);
    }

    /**
     * Turns the shooter (or robot if necessary) to face the given target, and fires using shooterPower().
     * @param target position to fire at
     * @param drive driving instance to turn robot if necessary (>150 degrees)
     */
    public void shoot(Vector2d target, StandardMechanumDrive drive) {
        if (shoot != null && turn != null) {
            Pose2d pose = localizer.getPoseEstimate();
            double bearing = target.angleBetween(pose.vec()) + pose.getHeading(); // should be + or -?
            if (Math.abs(bearing) > 150)
                turnShooter(bearing);
//                turn.setPosition(bearing); // TODO: Figure out how to translate radians to turn into turn pos for servo
            else drive.turn(bearing);
            shoot.setPower(shooterPower());
            linearOpMode.sleep(2000);
            shoot.setPower(0);
        }
    }

    public void grabWobble() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_GRAB);
    }

    public void releaseWobble() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_RELEASE);
    }

    public void wobbleArmDown() {
        if(wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_UP);
    }
    public void wobbleArmUp() {
        if(wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_DOWN);
    }

    /**
     * From -150 (all the way left) to 150 (all the way right) degrees.
     */
    void turnShooter(double angle) {
        final int TURN_OFFSET = 150;
        double maxBearing = 300;
        turn.setPosition(angle + TURN_OFFSET);
    }

    /**
     * Using the current voltage and the distance to the target on the field, calculate power necessary to get in the goal
     * @param target position on the field to calculate the distance.
     * @return
     */
    double shooterPower(Vector2d target) {
        double distance = localizer.getPoseEstimate().vec().distTo(target);
        double batteryVoltage = this.voltage.getVoltage();
        return 0;
    }

    double shooterPower() {
        return shooterPower(redGoal);
    }
}
