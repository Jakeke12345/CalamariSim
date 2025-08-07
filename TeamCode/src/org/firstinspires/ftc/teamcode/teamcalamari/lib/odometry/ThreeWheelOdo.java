package org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Pos2D;

/**
 * Used to track robots with three dead wheel odometry.
 *
 * @rep {Heading: Angle, Position: Pos2D}
 */
public class ThreeWheelOdo extends Odometry{

    /**
     * ThreeWheelOdo.Constants inner class used to track useful physical info.
     */
    public static class Constants{
        private final DistanceMeasure Ry;
        private final DistanceMeasure Ly;
        private final DistanceMeasure Bx;
        private final DistanceMeasure wheelDiameter;
        private final double ticksPerRev;
        private final double gearRatio;

        /**
         * Create a new instance of ThreeWheelOdo.Constants.
         * Params Ry, Ly, and Bx should be negative for left/down distances.
         *
         * @param Ry the horizontal distance from the center of the robot to the right wheel.
         *           (right wheel should be centered top/bottom)
         * @param Ly the horizontal distance from the center of the robot to the left wheel.
         *           (left wheel should be centered top/bottom)
         * @param Bx the vertical distance from the center of the robot to the x wheel.
         *           (x wheel should be centered left/right)
         * @param wheelDiameter the diameter of the odometry wheels.
         * @param ticksPerRev the # of encoder ticks in a wheel rotation.
         * @param gearRatio the gear ratio applied after ticksPerRev was counted.
         */
        public Constants(DistanceMeasure Ry, DistanceMeasure Ly, DistanceMeasure Bx,
                         DistanceMeasure wheelDiameter, double ticksPerRev, double gearRatio){

            this.Ry = Ry;
            this.Ly = Ly;
            this.Bx = Bx;
            this.wheelDiameter = wheelDiameter;
            this.ticksPerRev = ticksPerRev;
            this.gearRatio = gearRatio;
        }
    }

    private final Constants constants;

    /**
     * Create a new instance of ThreeWheelOdo.
     *
     * @param constants used to track useful physical info for calculations.
     * @param enc_r the motor corresponding to the right encoder.
     * @param enc_l the motor corresponding to the left encoder.
     * @param enc_b the motor corresponding to the x encoder.
     *
     * @rep {Heading: 0, Position: (0, 0)}
     */
    public ThreeWheelOdo(Constants constants, DcMotor enc_r, DcMotor enc_l, DcMotor enc_b){
        super(new DcMotor[]{enc_r, enc_l, enc_b});
        this.constants = constants;

        for(DcMotor encoder : encoders){
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void update() {
        int[] deltas = new int[3];
        for (int i=0; i<3; i++) {
            deltas[i] = encoders[i].getCurrentPosition() - lastPositions.get(i);
            lastPositions.set(i, lastPositions.get(i) + deltas[i]);
        }

        double ticks2inch = Math.PI * constants.wheelDiameter.inches() * constants.gearRatio / constants.ticksPerRev;
        double fwd   = ticks2inch * (deltas[0] * constants.Ly.inches() - deltas[1] * constants.Ry.inches()) / (constants.Ly.inches() - constants.Ry.inches());
        double theta = ticks2inch * (deltas[0] - deltas[1]) / (constants.Ly.inches() - constants.Ry.inches());
        double str   = ticks2inch * deltas[2] - constants.Bx.inches() * theta;

        heading = heading.plus(Angle.rads(theta));

        Pos2D delta_pos = Pos2D.inches(
                fwd * Math.cos(heading.rads()) - str * Math.sin(heading.rads()),
                str * Math.cos(heading.rads()) + fwd * Math.sin(heading.rads())
        );

        position = position.plus(delta_pos);
    }
}
