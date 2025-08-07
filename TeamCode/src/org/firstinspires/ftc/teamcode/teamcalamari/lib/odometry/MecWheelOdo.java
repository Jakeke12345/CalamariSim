package org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Pos2D;

/**
 * Used to track robots with mecanum encoder odometry.
 *
 * @rep {Heading: Angle, Position: Pos2D}
 */
public class MecWheelOdo extends Odometry{

    /**
     * MecWheelOdo.Constants inner class used to track useful physical info.
     */
    public static class Constants{
        private final DistanceMeasure wheelDiameter;
        private final double ticksPerRev;
        private final double gearRatio;

        /**
         * Create a new instance of MecWheelAuto.Constants.
         *
         * @param wheelDiameter the diameter of the mecanum wheels.
         * @param ticksPerRev the # of encoder ticks in a wheel rotation.
         * @param gearRatio the gear ratio applied after ticksPerRev was counted.
         */
        public Constants(DistanceMeasure wheelDiameter, double ticksPerRev, double gearRatio){
            this.wheelDiameter = wheelDiameter;
            this.ticksPerRev = ticksPerRev;
            this.gearRatio = gearRatio;
        }
    }

    private final Constants constants;
    private final IMU imu;

    /**
     * Create a new instance of MecWheelOdo.
     *
     * @param constants used to track useful physical info for calculations.
     * @param motors an array of the wheel motors, starting with leftback and going clockwise.
     *
     * @rep {Heading: 0, Position: (0, 0)}
     */
    public MecWheelOdo(Constants constants, DcMotor[] motors, IMU imu){
        super(motors);
        this.constants = constants;
        this.imu = imu;

        for(DcMotor encoder : encoders){
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void update() {
        int[] deltas = new int[4];
        for (int i=0; i<4; i++) {
            deltas[i] = encoders[i].getCurrentPosition() - lastPositions.get(i);
            lastPositions.set(i, lastPositions.get(i) + deltas[i]);
        }

        double ticks2inch = Math.PI * constants.wheelDiameter.inches() * constants.gearRatio / constants.ticksPerRev;
        double fwd = ticks2inch * (-deltas[0]-deltas[1]+deltas[2]+deltas[3]) / 4.0;
        double str = ticks2inch * (deltas[0]-deltas[1]-deltas[2]+deltas[3]) / 4.0;

        heading = Angle.rads(-imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        Pos2D delta_pos = Pos2D.inches(
                fwd * Math.cos(heading.rads()) - str * Math.sin(heading.rads()),
                str * Math.cos(heading.rads()) + fwd * Math.sin(heading.rads())
        );

        position = position.plus(delta_pos);
    }
}
