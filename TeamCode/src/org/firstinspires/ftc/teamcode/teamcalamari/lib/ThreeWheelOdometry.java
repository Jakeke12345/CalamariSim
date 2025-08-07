package org.firstinspires.ftc.teamcode.teamcalamari.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ThreeWheelOdometry {

    public static class Constants{
        private final DistanceMeasure Ry;
        private final DistanceMeasure Ly;
        private final DistanceMeasure Bx;
        private final DistanceMeasure wheelDiameter;
        private final double ticksPerRev;
        private final double gearRatio;

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

    public final Constants constants;
    public final DcMotor[] encoders;

    private final int[] lastPositions = {0,0,0};
    private Pos2D position = Pos2D.ZERO;
    private Angle heading = Angle.ZERO;

    public ThreeWheelOdometry(Constants constants, DcMotor enc_r, DcMotor enc_l, DcMotor enc_b){
        this.constants = constants;
        this.encoders = new DcMotor[]{enc_r, enc_l, enc_b};

        for(DcMotor encoder : encoders){
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void update() {
        int[] deltas = new int[3];
        for (int i=0; i<3; i++) {
            deltas[i] = encoders[i].getCurrentPosition() - lastPositions[i];
            lastPositions[i] += deltas[i];
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

    public Pos2D getPosition() { return position; }
    public Angle getHeading()  { return heading;  }
}
