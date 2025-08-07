package org.firstinspires.ftc.teamcode.teamcalamari.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry.Odometry;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry.ThreeWheelOdo;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.robot.MecBot;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.DistanceMeasure;

@TeleOp(group = "Test", name = "Odometry3")
public class TestOdo3 extends OpMode {
    String[] wheelNames = {"leftback", "leftfront","rightfront","rightback"};
    MecBot robot;
    Odometry odo;

    @Override
    public void init() {
        robot = new MecBot(wheelNames, MecBot.LINCOLN_MODE, this);

        // init odometry
        DcMotor[] wheels = robot.getWheelMotors();
        odo = new ThreeWheelOdo(
                new ThreeWheelOdo.Constants(
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
    public void loop() {
        odo.update();
        robot.update();

        telemetry.addData("Position", odo.getPosition());
        telemetry.addData("Heading", odo.getHeading());
        telemetry.update();
    }
}
