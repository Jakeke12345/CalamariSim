package org.firstinspires.ftc.teamcode.teamcalamari.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.ThreeWheelOdometry;

import java.util.Map;

@Autonomous(group = "Test", name = "Encoders")
public class TestEnc extends LinearOpMode {
    String[] wheelNames = {"leftback", "leftfront","rightfront","rightback"};
    final Map<String, double[]> WHEEL_POWERS = Map.of(
            "forward", new double[]{-1, -1, 1, 1},
            "right", new double[]{1, -1, -1, 1},
            "topOrbit", new double[]{1, 0, 0, 1},
            "bottomOrbit", new double[]{0, -1, -1, 0}
    );
    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
        // init motors
        for(int i=0; i<4; i++) wheels[i] = hardwareMap.dcMotor.get(wheelNames[i]);

        waitForStart();

        while(opModeIsActive()){
            // set power and telemetry
            for(int i=0; i<4; i++){
                wheels[i].setPower(WHEEL_POWERS.get("forward")[i]);
                telemetry.addData("Motor "+i, wheels[i].getCurrentPosition());
            }
            telemetry.update();

        }
    }
}
