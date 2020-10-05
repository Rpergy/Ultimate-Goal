package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {
    private TensorFlowRingDetection ringDetection;
    private String ringCase  = "";
    @Override
    public void runOpMode() throws InterruptedException {
        ringDetection = new TensorFlowRingDetection(this);
        if(isStopRequested()) return;
        waitForStart();
        ringCase = ringDetection.res(this);

    }
    private void getTrajectory(String ringCase) {
        switch (ringCase) {
            case "none":

                break;
            case TensorFlowRingDetection.LABEL_FIRST_ELEMENT:

                break;
            case TensorFlowRingDetection.LABEL_SECOND_ELEMENT:

                break;

            default:
                return;
        }
    }
}
