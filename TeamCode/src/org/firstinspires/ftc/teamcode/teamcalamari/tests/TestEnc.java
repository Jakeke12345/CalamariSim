package org.firstinspires.ftc.teamcode.teamcalamari.tests;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Map;

@Autonomous(group = "Test", name = "Encoders")
public class TestEnc extends LinearOpMode {
    String[] wheelNames = {"leftback", "leftfront","rightfront","rightback"};
    final Map<String, double[]> WHEEL_POWERS = Map.of(
            "forward", new double[]{-1, -1, 1, 1},
            "right", new double[]{1, -1, -1, 1},
            "topOrbit", new double[]{1, 0, 0, 1},
            "bottomOrbit", new double[]{0, -1, -1, 0},
            "turnCW", new double[]{-1, -1, -1, -1}
    );
    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
        // init motors
        for(int i=0; i<4; i++) wheels[i] = hardwareMap.dcMotor.get(wheelNames[i]);

        IMU imu = hardwareMap.get(BNO055IMUNew.class, "imu");

        waitForStart();

        while(opModeIsActive()){

            for(int i=0; i<4; i++){
                wheels[i].setPower(WHEEL_POWERS.get("turnCW")[i]);
                telemetry.addData("Motor "+i, wheels[i].getCurrentPosition());
            }
            telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();

        }
    }
}
