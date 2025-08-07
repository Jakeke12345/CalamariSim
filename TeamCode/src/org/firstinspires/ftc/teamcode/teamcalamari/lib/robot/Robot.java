package org.firstinspires.ftc.teamcode.teamcalamari.lib.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public abstract class Robot {

    protected final OpMode opMode;
    private final List<DcMotor> wheels = new ArrayList<>();

    public Robot(String[] wheelNames, OpMode opMode){
        this.opMode = opMode;
        for (String wheelName : wheelNames) {
            wheels.add(opMode.hardwareMap.dcMotor.get(wheelName));
        }
    }

    public void update(){
        // Auto only
        if(opMode instanceof LinearOpMode){


        // Teleop only
        } else {
            driveByControls();
        }
    }

    public void stop(){
        for(DcMotor wheel : wheels){
            wheel.setPower(0);
        }
    }

    protected abstract void driveByControls();

    protected void setWheelPowers(double... powers){
        // normalize
        double max = 1.0;
        for (int i=0; i<wheels.size(); i++){
            max = Math.max(max, powers[i]);
        }

        // set
        for (int i=0; i<wheels.size(); i++){
            wheels.get(i).setPower(powers[i]/max);
        }
    }

    public DcMotor[] getWheelMotors(){
        return wheels.toArray(new DcMotor[0]);
    }
}
