package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.ActuationConstants;
import org.firstinspires.ftc.teamcode.core.DriveConstants;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.*;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    /*
        Routine:
        1. Perform CV Operations. Will tell us which case we need to pursue for the rest of autonomous.
        2. Shoot preloaded rings at power shots.
        3. Collect rings (if applicable).
        4. Shoot rings into top most goal (if applicable).
        4. Go to corresponding square, drop wobble goals in said squares.
        5. Park.
     */

    private TensorFlowRingDetection ringDetection;
    private String ringCase = "";
    private StandardMechanumDrive drive;
    Actuation actuation;

    Vector2d centerA = new Vector2d(12,-60);
    Vector2d centerB = new Vector2d(36,-36);
    Vector2d centerC = new Vector2d(60, -60);
    Pose2d startPose = new Pose2d(-63, -36);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new StandardMechanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        actuation = new Actuation(this, drive.getLocalizer());

//        ringDetection = new TensorFlowRingDetection(this);

        waitForStart();
        ringCase = "Quad"; //ringDetection.res(this); //TODO: Hardcoding for now. Change when camera is mounted
        telemetry.addData("Ring case", ringCase);
        telemetry.update();

        if (isStopRequested()) return;
        powerShots();
        getCaseTrajectory(ringCase);
        park();
    }

    private void powerShots() {
        actuation.shoot(POWER_SHOT_LEFT, drive);
        actuation.shoot(POWER_SHOT_MIDDLE, drive);
        actuation.shoot(POWER_SHOT_RIGHT, drive);
    }

    void park() {
        Pose2d pose = drive.getPoseEstimate();
        drive.followTrajectory(drive.trajectoryBuilder(pose).lineToConstantHeading(new Vector2d(12, pose.getY())).build());
    }

    /**
        1. Go to center square of desired square
        2. Release wobble
        3. Go to start using pathToStart()
        4. Grab other wobble
        5. Go back to same case square
        6. Release
     */
    void wobbleRoutine(Vector2d center) {
        Pose2d centerPose = new Pose2d(center.getX(), center.getY(), 0);
        Pose2d backPose = new Pose2d(-55,-48, toRadians(-90));
        drive.followTrajectory(drive.staticSpline(center).build());
        actuation.releaseWobble();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(90))
                .splineToSplineHeading(backPose, toRadians(45))
                .build());
//        drive.trajectoryBuilder()
//        drive.followTrajectory(drive.staticSpline(new Vector2d(-55, -48), -90).build());
        actuation.grabWobble();
        drive.followTrajectory(drive.trajectoryBuilder(backPose, toRadians(0))
                .splineToSplineHeading(centerPose, toRadians(0)).build());
    }

    private void getCaseTrajectory(String ringCase) {
        Vector2d ringPos = new Vector2d(-24, -36);
        Trajectory startToRings = drive.trajectoryBuilder(startPose).splineToConstantHeading(ringPos, 0).build();
        switch (ringCase) {
            case "None": // Zero rings, case "A"
                wobbleRoutine(centerA);
                break;

            case TensorFlowRingDetection.LABEL_SECOND_ELEMENT: // One ring, case "B", "Single"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();
                wobbleRoutine(centerB);
                break;

            case TensorFlowRingDetection.LABEL_FIRST_ELEMENT: // 4 rings, case "C", "Quad"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();
                wobbleRoutine(centerC);
                break;
        }
    }
}
