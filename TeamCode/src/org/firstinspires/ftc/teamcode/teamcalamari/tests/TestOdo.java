package org.firstinspires.ftc.teamcode.teamcalamari.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.ThreeWheelOdometry;

@TeleOp(group = "Test", name = "Odometry")
public class TestOdo extends OpMode {
    String[] wheelNames = {"leftback", "leftfront","rightfront","rightback"};

    DcMotor[] wheels = new DcMotor[4];

    ThreeWheelOdometry odo;

    @Override
    public void init() {
        // init motors
        for (int i = 0; i < 4; i++) wheels[i] = hardwareMap.dcMotor.get(wheelNames[i]);

        // init odometry
        odo = new ThreeWheelOdometry(
                new ThreeWheelOdometry.Constants(
                        // Ry, Ly, Bx
                        DistanceMeasure.inches(6),
                        DistanceMeasure.inches(-6),
                        DistanceMeasure.inches(6),

                        DistanceMeasure.inches(2),
                        MotorType.Neverest40.TICKS_PER_ROTATION, 1
                ),
                wheels[0], wheels[1], wheels[2]
        );
    }
    @Override
    public void loop(){
        odo.update();

        // Teleop controls
        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;
        double pa = gamepad1.right_stick_x;

        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        wheels[0].setPower(-p1);
        wheels[1].setPower(-p2);
        wheels[2].setPower(p3);
        wheels[3].setPower(p4);

        // Telemetry
        telemetry.addData("Position", odo.getPosition());
        telemetry.addData("Heading", odo.getHeading());
        telemetry.update();
    }
}
