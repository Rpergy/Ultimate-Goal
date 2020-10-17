package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ValueCycler {
    double[] vals;
    int index = 0;
    Gamepad gamepad;
    CONTROL_PAIRS pair;
    GamepadEventPS update;
    double currentVal;

    public ValueCycler(Gamepad gamepad, double[] vals, CONTROL_PAIRS pairs, GamepadEventPS update) {
        this.gamepad = gamepad;
        this.vals = vals;
        pair = pairs;
        this.update = update;
    }

    public double update() {
        switch (pair) {
            case BUMPERS:
                increment(update.rightBumper());
                decrement(update.leftBumper());
                break;
            case D_PAD_VERTICAL:
                increment(update.dPadUp());
                decrement(update.dPadDown());
                break;

            case D_PAD_HORIZONTAL:
                increment(update.dPadRight());
                decrement(update.dPadLeft());
                break;
        }
        return currentVal;
    }

    private void decrement(boolean condition) {
        if (condition) {
            if (index - 1 >= 0) {
                index--;
                currentVal = vals[index];
            }
        }
    }

    private void increment(boolean condition) {
        if (condition) {
            if (vals.length - 1 > index + 1) {
                index++;
                currentVal = vals[index];
            }
        }
    }


    public enum CONTROL_PAIRS {D_PAD_VERTICAL, D_PAD_HORIZONTAL, BUMPERS}
}
