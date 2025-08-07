package org.firstinspires.ftc.teamcode.teamcalamari.lib.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.Control;

public class MecBot extends Robot {

    public static class MecControls {
        private final Control<Float> fwd;
        private final Control<Float> str;
        private final Control<Float> tws;

        public MecControls(Control<Float> fwd, Control<Float> str, Control<Float> tws){
            this.fwd = fwd;
            this.str = str;
            this.tws = tws;
        }
    }

    public static final MecControls LINCOLN_MODE = new MecControls(
            new Control<>("left_stick_y", "gamepad1"),
            new Control<>("left_stick_x", "gamepad1"),
            new Control<>("right_stick_x", "gamepad1")
    );

    private final MecControls controls;

    public MecBot(String[] wheelNames, MecControls controls, OpMode opMode){
        super(wheelNames, opMode);
        this.controls = controls;
    }

    @Override
    protected void driveByControls(){
        double px = controls.str.getValue(opMode);
        double py = -controls.fwd.getValue(opMode);
        double pa = -controls.tws.getValue(opMode);
        setWheelPowers(
                px - py + pa,
                -px - py + pa,
                -px + py + pa,
                px + py + pa
        );
    }

    @Override
    public void update(){
        super.update();
    }

    @Override
    public void stop(){
        super.stop();
    }

}
