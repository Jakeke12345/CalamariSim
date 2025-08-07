package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import virtual_robot.controller.BotConfig;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, a Servo-controlled arm on the back, and three dead-wheel encoder pods
 *
 * MecanumBot is the controller class for the "mecanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "Calamari Bot 2", filename = "calamari_bot2")
public class CalamariBot2 extends MecanumPhysicsBase {

    //private ServoImpl servo = null;

    private MotorType encoderMotorType;
    private DeadWheelEncoder yEncoder, xEncoder, nothingEncoder = null;

    // backServoArm is instantiated during loading via a fx:id property.
    //@FXML Rectangle backServoArm;

    //Dimensions in inches for encoder wheels.
    //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
    //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double Y_ENCODER_X = 0.0;
    //private final double RIGHT_ENCODER_X = 6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = 0.0;

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double yEncoderX;
    private double xEncoderY;

    public CalamariBot2(){
        super(false, false,
                new String[]{"leftback", "leftfront","rightfront","rightback"}
        );
    }

    public void initialize(){
        //servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        //leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        //rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        //xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");

        encoderMotorType = MotorType.Neverest40;

        yEncoder = new DeadWheelEncoder(encoderMotorType, motorController1, 0);
        //leftEncoder = new DeadWheelEncoder(encoderMotorType, motorController1, 1);
        xEncoder = new DeadWheelEncoder(encoderMotorType, motorController1, 1);
        nothingEncoder = new DeadWheelEncoder(encoderMotorType, motorController1, 2);

        setEncoders(xEncoder, yEncoder, nothingEncoder, nothingEncoder);
        super.initialize();

        //octoQuad.setEncoder(4, leftEncoder);
        //octoQuad.setEncoder(5, rightEncoder);
        //octoQuad.setEncoder(6, xEncoder);

        //Dimensions in pixels
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        yEncoderX = Y_ENCODER_X * botWidth / 18.0;
        //rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        hardwareMap.setActive(false);
        //backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    /*
    protected void createHardwareMap(){
        super.createHardwareMap();
    }
     */

    public synchronized void updateStateAndSensors(double millis){

        //Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        //Compute new pose and update various sensors
        super.updateStateAndSensors(millis);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double yEncoderRadians = (dyR + yEncoderX * headingChange) / encoderWheelRadius;
        //double leftEncoderRadians = (dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = (dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        yEncoder.update(yEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        //((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
        yEncoder.stopAndReset();
        xEncoder.stopAndReset();
    }


}
