package org.firstinspires.ftc.teamcode.core

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS

@TeleOp(name = "Shooter Velocity Test")
class ShooterVelocityTest : OpMode() {
    fun slope(x2: Double, x1: Double, y2: Double, y1: Double): Double = (y2 - y1) / (x2 - x1)

    lateinit var shooter : DcMotor
    lateinit var update : GamepadEventPS
    var power = 0.0
    val posVals = mutableMapOf<Double, Double>()
    var currentPower = 0.0
    val slopes = mutableListOf<Double>()

    override fun init() {
        update = GamepadEventPS(gamepad1)
        shooter = hardwareMap.dcMotor["shooter"]
        shooter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    @RequiresApi(Build.VERSION_CODES.N)
    override fun loop() {
        if (update.dPadLeft()) {
            if (Math.abs(power) >= 0.01) power -= .01
        }

        if (update.dPadRight()) {
            if (Math.abs(power) <= 1) power += .01
        }

        if (shooter.power != 0.0) {
            if (shooter.power == currentPower) {
                posVals.put(time, shooter.currentPosition.toDouble())
                if (posVals.size >= 100) {
                    val sorted = posVals.toSortedMap()
                    val keys = sorted.keys.toDoubleArray()
                    val vals = sorted.values.toDoubleArray()
                    for(i in 1..sorted.size) {
                        slopes.add(slope(x2 = keys[i], x1 = keys[i - 1], y2 = vals[i], y1 = vals[i - 1]))
                    }
                    telemetry.addData("Slopes SD", sd(slopes))
                    telemetry.addData("Average slope", slopes.average())
                    telemetry.update()
                }
            } else {
                currentPower = shooter.power
                posVals.clear()
            }
        }


        shooter.power = power
        telemetry.addData("Shooter power", power)
        telemetry.addData("square to put in feed pos", "triangle to put in initial pos")
        telemetry.update()

    }

    fun sd(numArray: MutableList<Double>): Double {
        var sum = 0.0
        var standardDeviation = 0.0
        for (num in numArray) {
            sum += num
        }

        val mean = sum / 10

        for (num in numArray) {
            standardDeviation += Math.pow(num - mean, 2.0)
        }
        return Math.sqrt(standardDeviation / 10)
    }
}