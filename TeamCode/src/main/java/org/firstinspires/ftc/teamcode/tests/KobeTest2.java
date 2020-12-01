package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static java.lang.Math.*;

@TeleOp(name = "Kobe Test Part 2")
public class KobeTest2 extends OpMode {
    DcMotorEx shooter;
    final double wheelRadius = 0.0508; // in meters
    double angularVelocity = 0; // rad/s
    double linearVelocity = 0; // m/s

    final double MAX_RPM = 6000.0; // in RPM, if you couldn't figure it out
    final double MAX_RAD = MAX_RPM * ((2.0 * PI) / 60.0);
    final double MAX_LINEAR_SPEED = MAX_RAD * wheelRadius; // m/s

    GamepadEventPS update;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        update = new GamepadEventPS(gamepad1);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1,0,0,.1));
//        shooter.setVelocityPIDFCoefficients(1,0,0,0); //same thing
//        shooter.setPower(1);
    }

    @Override
    public void loop() {

        if(update.dPadUp())
            angularVelocity += .5;
        if(update.dPadDown())
            angularVelocity -= .5;

        linearVelocity = angularVelocity * wheelRadius;

        telemetry.addData("Set velocity (ticks/s)", shooter.getVelocity());
        shooter.setVelocity(angularVelocity, AngleUnit.RADIANS);
        telemetry.addData("Use dPad Up/Down to change motor speed", "");
        telemetry.addData("Angular velocity (rad)", angularVelocity);
        telemetry.addData("Angular Velocity (deg)", toDegrees(angularVelocity));
        telemetry.addData("Linear Velocity", linearVelocity);
        telemetry.addData("Motor Power", shooter.getPower());

        telemetry.update();
    }

    double kinematicSolver(double x) { // in meters
        double Vix = 20.0; // m/s, for now choose any value
//        double x = 3.204; // in meters
        double y = .5; // in meters
        double t = x / Vix;
        double heading = 16.62; // degrees
        double Viy = y + .5 * 9.8 * pow(t, 2);
        double wheelRadius = .1; // meters
        double Vi = sqrt(pow(Viy, 2) + pow(Vix, 2));
        double angularSpeed = Vi / wheelRadius;
        return angularSpeed;
    }
}
