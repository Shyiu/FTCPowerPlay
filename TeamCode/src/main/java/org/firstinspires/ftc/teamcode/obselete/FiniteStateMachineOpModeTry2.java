package org.firstinspires.ftc.teamcode.obselete;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBot;

@Disabled
@TeleOp(group = "Mecanum Tele-Op", name = "Mecanum Powerplay Teleop")
public class FiniteStateMachineOpModeTry2 extends OpMode {
    //

    public enum CommandState {
        MOVE_ARM,
        MOVE_JOINT,
        MOVE_CLAW
    };

    CommandState state = CommandState.MOVE_ARM;


    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
    protected NormalizedColorSensor color;

    final int[] LEVELS = {86, 1008, 1540};
    final double[] SERVO_POS = {.58,1,.22,0};
    int servo_position = 0;
    int current_level = 0;

    double clawOpen = .78;
    double clawClose = 1;
    double armJoint2Max = 1;
    double armJoint2Min = 0;
    double armJoint2Increment = .025;
    double y2;
    double incrementWait = .5;


    double armJoint2CurrentPos = 0;
    double armJoint1CurrentPos = 0;
    double armJoint1Min = 0;

    DcMotor armJoint1;
    Servo armJoint2;
    Servo claw;
    BNO055IMU imu;
    MecanumBot m = new MecanumBot();
    @Override
    public void init() {


        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, m.fr);
        frontLeft = hardwareMap.get(DcMotor.class, m.fl);
        backRight = hardwareMap.get(DcMotor.class, m.br);
        backLeft = hardwareMap.get(DcMotor.class, m.bl);

        slides = hardwareMap.get(DcMotor.class, m.slides);
        color = hardwareMap.get(NormalizedColorSensor.class, m.color);
        imu = hardwareMap.get(BNO055IMU.class, m.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);



        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        color.setGain((float)15);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < .52 && colors.red < .52){
            slides.setPower(.3);
            colors = color.getNormalizedColors();
        }


    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);






            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed



            // Makes the Driver Hub output the message
            // Left Target Power: A float from [-1,1]
            // Right Target Power: A float from [-1,1]
            // The current power of each motor


            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());




            // Status: Running
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
}
