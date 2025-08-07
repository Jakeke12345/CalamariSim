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
 * Used to track robots with two dead wheel odometry and the IMU.
 *
 * @rep {Heading: Angle, Position: Pos2D}
 */
public class TwoWheelOdo extends Odometry{

    /**
     * TwoWheelOdo.Constants inner class used to track useful physical info.
     */
    public static class Constants{
        private final DistanceMeasure wheelDiameter;
        private final double ticksPerRev;
        private final double gearRatio;

        /**
         * Create a new instance of TwoWheelOdo.Constants.
         * The x encoder should be centered top/bottom, the y encoder should be centered L/R
         *
         * @param wheelDiameter the diameter of the odometry wheels.
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
     * Create a new instance of ThreeWheelOdo.
     *
     * @param constants used to track useful physical info for calculations.
     * @param enc_x the motor corresponding to the horizontal encoder.
     * @param enc_y the motor corresponding to the vertical encoder.
     *
     * @rep {Heading: 0, Position: (0, 0)}
     */
    public TwoWheelOdo(Constants constants, DcMotor enc_x, DcMotor enc_y, IMU imu){
        super(new DcMotor[]{enc_x, enc_y});
        this.constants = constants;
        this.imu = imu;

        for(DcMotor encoder : encoders){
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void update() {
        int[] deltas = new int[2];
        for (int i=0; i<2; i++) {
            deltas[i] = encoders[i].getCurrentPosition() - lastPositions.get(i);
            lastPositions.set(i, lastPositions.get(i) + deltas[i]);
        }

        double ticks2inch = Math.PI * constants.wheelDiameter.inches() * constants.gearRatio / constants.ticksPerRev;
        double fwd = ticks2inch * deltas[1];
        double str = ticks2inch * deltas[0];

        heading = Angle.rads(-imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

        Pos2D delta_pos = Pos2D.inches(
                fwd * Math.cos(heading.rads()) - str * Math.sin(heading.rads()),
                str * Math.cos(heading.rads()) + fwd * Math.sin(heading.rads())
        );

        position = position.plus(delta_pos);
    }
}
