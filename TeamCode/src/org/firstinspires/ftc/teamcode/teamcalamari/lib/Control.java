package org.firstinspires.ftc.teamcode.teamcalamari.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

public class Control<T> {

    Field control;
    Field gamepad;

    public Control(String control, String gamepad){
        try {
            this.gamepad = OpMode.class.getField(gamepad);
            this.control = Gamepad.class.getField(control);

        } catch (NoSuchFieldException e) {
            throw new RuntimeException("Control must be created with valid args");
        }
    }

    public T getValue(OpMode opMode){
        try {
            return (T) control.get(gamepad.get(opMode));
        } catch (IllegalAccessException | IllegalArgumentException e) {
            throw new RuntimeException(e);
        }
    }

}
