package org.firstinspires.ftc.teamcode.teamcalamari.lib.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.lib.util.Pos2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract class for all odometry configurations.
 *
 * @rep {Heading: Angle, Position: Pos2D}
 */
public abstract class Odometry {

    protected final DcMotor[] encoders;
    protected final List<Integer> lastPositions = new ArrayList<>();
    protected Pos2D position = Pos2D.ZERO;
    protected Angle heading = Angle.ZERO;

    /**
     * Create a new instance of Odometry.
     *
     * @param encoders a list of motors corresponding to relevant encoders.
     *
     * @rep {Heading: 0, Position: (0, 0)}
     */
    public Odometry(DcMotor[] encoders){
        this.encoders = encoders;
        for (DcMotor encoder : encoders){
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lastPositions.add(0);
        }
    }

    /**
     * Call in loop to track position.
     *
     * @updates this.Position, this.Heading
     */
    public abstract void update();

    /**
     * Returns the robot position.
     * @return this.Position
     */
    public Pos2D getPosition() { return position; }

    /**
     * Returns the robot heading.
     * @return this.Heading
     */
    public Angle getHeading()  { return heading;  }
}
