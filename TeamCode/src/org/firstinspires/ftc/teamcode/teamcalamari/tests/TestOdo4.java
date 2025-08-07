package org.firstinspires.ftc.teamcode.teamcalamari.tests;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry.MecWheelOdo;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry.Odometry;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry.TwoWheelOdo;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.robot.MecBot;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.DistanceMeasure;

@TeleOp(group = "Test", name = "Odometry4")
public class TestOdo4 extends OpMode {
    String[] wheelNames = {"leftback", "leftfront","rightfront","rightback"};
    MecBot robot;
    Odometry odo;

    @Override
    public void init() {
        robot = new MecBot(wheelNames, MecBot.LINCOLN_MODE, this);

        // init odometry
        DcMotor[] wheels = robot.getWheelMotors();
        IMU imu = hardwareMap.get(BNO055IMUNew.class, "imu");

        odo = new MecWheelOdo(
                new MecWheelOdo.Constants(
                        DistanceMeasure.inches(4),
                        MotorType.Neverest40.TICKS_PER_ROTATION, 1
                ),
                wheels, imu
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
