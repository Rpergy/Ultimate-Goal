//RP Import all necessary packages
package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_END;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_RESTING;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FLYWHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.RESTING_TURNING_POS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.TOWER_GOAL_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_MIDDLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_DOWN;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_UP;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_GRAB;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_RELEASE;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.redGoal;

public class Actuation {
    //RP specifying the components
    DcMotorEx shoot;
    DcMotor intake;
    Servo turn, wobbleGrab, wobbleArm, feeder;
    HardwareMap hardwareMap;
    Localizer localizer;
    VoltageSensor voltage;
    LinearOpMode linearOpMode = null;
    RevColorSensorV3 colorsensor;

    /**
     * For Autonomous initialization specifically.
     */
    public Actuation(LinearOpMode linearOpMode, Localizer localizer) {
        this(linearOpMode.hardwareMap, localizer, linearOpMode);
        this.linearOpMode = linearOpMode;
    }

    /**
     * For TeleOp initialization specifically.
     */
    public Actuation(HardwareMap hardwareMap, Localizer localizer, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;
        this.linearOpMode = linearOpMode;

        voltage = hardwareMap.voltageSensor.iterator().next();
        
        //RP setting all the modes and values of the components (grabber, shooter, etc.)
        if (hardwareMap.dcMotor.contains("intake")) {
            intake = hardwareMap.dcMotor.get("intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (hardwareMap.dcMotor.contains("shooter")) {
            shoot = hardwareMap.get(DcMotorEx.class, "shooter");
            shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot.setVelocityPIDFCoefficients(1, 1, 0, 1);
        }

        if (hardwareMap.servo.contains("wobbleGrab")) {
            wobbleGrab = hardwareMap.servo.get("wobbleGrab");
            if (linearOpMode != null)
                wobbleClawClose();
            else
                wobbleClawOpen();
        }

        if (hardwareMap.servo.contains("wobbleArm")) {
            wobbleArm = hardwareMap.servo.get("wobbleArm");
            wobbleArmUp();
        }

        if (hardwareMap.colorSensor.contains("colorSensor")) {
            colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        }

        if (hardwareMap.servo.contains("feeder")) {
            feeder = hardwareMap.servo.get("feeder");
            feeder.setPosition(FEEDER_RESTING);
        }
    }


    // All Shooter Operations

    public void feedRing() {
        feeder.setPosition(FEEDER_END);
        feeder.setPosition(FEEDER_RESTING);
    }

    public boolean hasRings() {
        if (colorsensor == null) return false;
        return Math.abs(colorsensor.getNormalizedColors().red - 170) > 20;
    }

    /**
     * Turns the shooter (or robot if necessary) to face the red goal, then fires.
     */
    public void shoot(StandardMechanumDrive drive) {
        shoot(TOWER_GOAL, drive);
    }

    /**
     * Turns the shooter (or robot if necessary) to face the given target, and fires using
     * calcInitialSpeed().
     *
     * @param target position to fire at
     * @param drive  driving instance to turn robot if necessary (>150 degrees)
     */
    public void shoot(Target target, StandardMechanumDrive drive) {
        if (shoot == null) return;
        Pose2d pose = localizer.getPoseEstimate();
        double bearing = target.pos().angleBetween(pose.vec()) + pose.getHeading(); // should be + or -?

        if (linearOpMode == null) { // If we are in TeleOp
            if (pose.getX() > SHOOT_LINE - 9) {
                drive.followTrajectory(
                        drive.trajectoryBuilder(pose)
                                .lineToLinearHeading(new Pose2d(SHOOT_LINE - 9, pose.getY(), bearing))
                                .build()
                );
            }
        } else {
            drive.turn(bearing);
        }
        feedRing();
        shoot.setVelocity(calcInitialSpeed(target), AngleUnit.RADIANS);
        if (linearOpMode != null)
            linearOpMode.sleep(2000); //TODO: Find appropriate delay, enough to let the motor shoot.
        shoot.setPower(0);

    }

    public void powerShots(StandardMechanumDrive drive) {
        shoot(POWER_SHOT_LEFT, drive);
        shoot(POWER_SHOT_MIDDLE, drive);
        shoot(POWER_SHOT_RIGHT, drive);
    }

    public void preheatShooter() {
        if (shoot != null)
            shoot.setVelocity(calcInitialSpeed(TOWER_GOAL), AngleUnit.RADIANS);
    }

    public void killFlywheel() {
        if (shoot != null)
            shoot.setPower(0);
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
     * Using a kinematics approach to calculate the correct initial velocity to launch the ring so
     * it can reach its goal, based on the vertical/horizontal distance from the target. See
     * https://www.desmos.com/calculator/qxdrswohm7 for more details, and the engineering notebook
     * for a complete derivation.
     *
     * @param target One of 4 targets to shoot at (3 power shots, and the tower goal)
     * @return Adjusted speed to run the motor at (see adjust() for more details on the offset).
     */
    double calcInitialSpeed(Target target) {
        double h = 0.0; // in meters
        final double g = 9.8; // In m/s^2
        double d = 0.0; // in meters
        switch (target) {
            case TOWER_GOAL:
                h = TOWER_GOAL_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(redGoal);
                break;
            case POWER_SHOT_LEFT:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(leftPowerShot);
                break;
            case POWER_SHOT_RIGHT:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(rightPowerShot);
                break;
            case POWER_SHOT_MIDDLE:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(centerPowerShot);
                break;
        }
        double linearSpeed = sqrt((pow(d, 2) * g) / (pow(cos(LAUNCHER_ANGLE), 2) * (2 * d * tan(LAUNCHER_ANGLE) - 2 * h)));
        return adjust(linearSpeed / FLYWHEEL_RADIUS);
    }

    /**
     * Offsets the calculated, "theoretical" initial angular velocity (in rad/s) based on testing.
     * When the launcher actually is fed a ring, its set velocity (which is accurate to about +/- .5
     * rad/s) decreases due to the resistance from the ring. So, based on some tests we ran, the
     * speed is offsetted by about .75 rad/s to counter the decrease to keep expectations
     * consistent.
     *
     * @param angVel Speed from calcInitialSpeed()
     * @return Adjusted speed, still in rad/s
     */
    double adjust(double angVel) {
        return angVel + .75;
    }


    // All Intake Operations

    public void suck() {
        if (intake != null)
            intake.setPower(1);
    }

    public void stopIntake() {
        if (intake != null)
            intake.setPower(0);
    }

    public void spitOut() {
        if (intake != null)
            intake.setPower(-1);
    }


    // All Wobble Operations

    public void wobbleClawClose() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_GRAB);
    }

    public void wobbleClawOpen() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_RELEASE);
    }

    public void wobbleArmDown() {
        if (wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_DOWN);
    }

    public void wobbleArmUp() {
        if (wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_UP);
    }

    public void wobbleArmSlightltyUp() {
        if(wobbleArm != null)
            wobbleArm.setPosition(.6);
    }

    public void placeWobble() {
        if (wobbleArm != null && wobbleGrab != null) {
            wobbleArmDown();
            linearOpMode.sleep(750);
            wobbleClawOpen();
            linearOpMode.sleep(750);
            wobbleArmUp();
        }
    }
    
    public void grabWobble() {
        if (wobbleArm != null && wobbleGrab != null) {
            wobbleArmDown();
            linearOpMode.sleep(750);
            wobbleClawClose();
            linearOpMode.sleep(750);
            wobbleArmUp();
        }
    }
    
    //RP Returns positions of wobble arm
    public boolean isWobbleArmUp() {
        return wobbleArm.getPosition() == WOBBLE_ARM_UP;
    }

    public boolean isWobbleArmDown() {
        return wobbleArm.getPosition() == WOBBLE_ARM_DOWN;
    }

    public boolean isWobbleClawOpen() {
        return wobbleGrab.getPosition() == WOBBLE_RELEASE;
    }

    public boolean isWobbleClawClosed() {
        return wobbleGrab.getPosition() == WOBBLE_GRAB;
    }

}
