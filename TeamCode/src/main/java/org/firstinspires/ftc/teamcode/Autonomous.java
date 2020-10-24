package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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
        actuation = new Actuation(this, drive.getLocalizer());
        ringDetection = new TensorFlowRingDetection(this);
        if (isStopRequested()) return;
        waitForStart();
        ringCase = ringDetection.res(this);
        powerShots();
        getCaseTrajectory(ringCase);
        park();
    }

    private void powerShots() {
        Vector2d leftPowerShotPos = new Vector2d(DriveConstants.MAX_EDGE, -4.5);
        Vector2d centerPowerShotPos = new Vector2d(DriveConstants.MAX_EDGE, -9);
        Vector2d rightPowerShotPos = new Vector2d(DriveConstants.MAX_EDGE, -13.5);
        Vector2d[] targets = {leftPowerShotPos, centerPowerShotPos, rightPowerShotPos};
        for (int i = 0; i < 3; i++) {
            actuation.shoot(targets[i]);
        }
    }

    void park() {
        Pose2d pose = drive.getPoseEstimate();
        drive.trajectoryBuilder(pose).lineToConstantHeading(new Vector2d(0, pose.getY()));
    }

    /*
        1. Go to center square of desired square
        2. Release wobble
        3. Go to start using pathToStart()
        4. Grab other wobble
        5. Go back to same case square
        6. Release
     */
    void wobbleRoutine(String caseLetter) {
        caseLetter = caseLetter.toUpperCase();
        switch (caseLetter) {
            case "A":
                wobbleBaseRoutine(centerA);
                break;
            case "B":
                wobbleBaseRoutine(centerB);
                break;
            case "C":
                wobbleBaseRoutine(centerC);
                break;
            default:
                // What the hell did you do
                break;
        }
    }

    void wobbleBaseRoutine(Vector2d center) {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(center, 0).build());
        actuation.releaseWobble();
        drive.followTrajectory(pathToStart().build());
        actuation.grabWobble();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(center, 0).build());
    }

    TrajectoryBuilder pathToStart() {
        return drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(startPose.vec(), 0);
    }

    private void getCaseTrajectory(String ringCase) {

        Vector2d ringPose = new Vector2d(-32, -36);
        Trajectory startToRings = drive.trajectoryBuilder(startPose).splineToConstantHeading(ringPose, 0).build();
        switch (ringCase) {
            case "none": // Zero rings, case "A"
                wobbleRoutine("A");
                break;
            case TensorFlowRingDetection.LABEL_FIRST_ELEMENT: // One ring, case "B"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();
                wobbleRoutine("B");

                break;
            case TensorFlowRingDetection.LABEL_SECOND_ELEMENT: // 4 rings, case "C"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();
                wobbleRoutine("C");
                break;
        }
    }
}
