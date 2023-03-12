package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Lift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@Config
@TeleOp(name = "Mecanum Power Play Teleop", group = "Tele-Op")
public class MecanumPowerPlayTeleOp extends LinearOpMode {
    private ElapsedTime time  = new ElapsedTime();
    private double slideTimeout;
    private double flapperTimeout = 0;
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    public double priorPosition = -1;
    protected Lift slides;
    public boolean testingIntake = true;//Disables Drive
    protected DcMotor tape;
    DcMotorSimple rightServo;
    DcMotorSimple leftServo;
    public static double MAX_SPEED = 1;
    public enum FLAPPER_STATE{
        MOVE,
        WAIT
    }
    FLAPPER_STATE flapperToggle = FLAPPER_STATE.WAIT;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    protected NormalizedColorSensor color;
    final double FLAPPER_TIME = 0.5; //Amount of time till flappers timeout
    public enum DRIVE_STATE{
        DRIVE_TANK,
        DRIVE_STRAFE,
        SCAN_TO_RIGHT,
        SCAN_TO_LEFT,
        WAIT
    }
    boolean closedPrior = false;
    public enum SLIDE_STATE{
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
        WAIT
    }

    public enum LIGHT_STATE{
        DEFAULT,
        PARK_ALERT,
        OVERRIDE_SLIDES,
        CONE_GRAB,
        VOLTAGE
    }
//    LIGHT_STATE led = LIGHT_STATE.DEFAULT;
    SLIDE_STATE slide_position = SLIDE_STATE.WAIT;
    //Between field centric, straightforward and my weird version
    double leftTgtPower = 0, rightTgtPower = 0;
    RevBlinkinLedDriver.BlinkinPattern park, override, cone, voltage;
//    RevBlinkinLedDriver ledDevice;
    public MecanumBotConstant names = new MecanumBotConstant();
    //60 is encoder position
    //Slide Related Variables
    double TOP_HARDSTOP = names.TOP_HARDSTOP;
    double BOTTOM_HARDSTOP = names.BOTTOM_HARDSTOP;//Actually supposed to be 0
    double[] SLIDE_POSITIONS = names.SLIDE_POSITIONS;
//    double[] STACK_POSITIONS = names.CONE_STACK_LEVELS;
    int conesOnStack = 5;

    int slideIndexObs = 0;
    int slideIndex = 0;
    boolean cone_stack = false;
    boolean toggleHardstops = false;
    //Flap related Variables
    final double flapUp = .379;
    final double flapDown = .77;

    OpenCvCamera junctionCam;
    JunctionPipeline junctionDetectionPipeline;
    public static double speed = .4;
    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        junctionCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, names.cameraJunction));
        junctionDetectionPipeline = new JunctionPipeline(telemetry);
        junctionCam.setPipeline(junctionDetectionPipeline);
        junctionCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                junctionCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, names.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        slides = new Lift(hardwareMap, 72, 16, 8);
        rightServo = hardwareMap.get(DcMotorSimple.class, names.right_servo);//change name to servo that is being tested.
        leftServo = hardwareMap.get(DcMotorSimple.class, names.left_servo);
        rightServo.setPower(names.rightServoOpenPosition);
        leftServo.setPower(names.leftServoOpenPosition);

        color = hardwareMap.get(NormalizedColorSensor.class, names.color);


        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

//        ledDevice = hardwareMap.get(RevBlinkinLedDriver.class, names.led);
//        voltage = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
//        park = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
//        cone = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
//        override = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();
        double LEDTIME = time.time();
        while (!isStopRequested() && opModeIsActive()) {
            switch (command) {
                case SCAN_TO_LEFT:
                    frontRight.setPower(speed);
                    backRight.setPower(speed);
                    frontLeft.setPower(-speed);
                    backLeft.setPower(-speed);
                    command = DRIVE_STATE.WAIT;
                    break;
                case SCAN_TO_RIGHT:
                    frontRight.setPower(-speed);
                    backRight.setPower(-speed);
                    frontLeft.setPower(speed);
                    backLeft.setPower(speed);
                    command = DRIVE_STATE.WAIT;
                    break;
                case WAIT:
                    if (gamepad1.y) {
                        command = DRIVE_STATE.DRIVE_STRAFE;
                        break;
                    }
                    if (junctionDetectionPipeline.getLocation() == null) {
                        command = DRIVE_STATE.DRIVE_TANK;
                        break;
                    }
                    switch (junctionDetectionPipeline.getLocation()) {
                        case TARGET:
                            telemetry.addLine("TARGET)");
                            frontLeft.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                            backRight.setPower(0);
                            command = DRIVE_STATE.DRIVE_TANK;
                            break;
                    }
                    break;
                case DRIVE_TANK:
                    double leftPower = sameSignSqrt(-gamepad1.left_stick_y);
                    double rightPower = sameSignSqrt(-gamepad1.right_stick_y);
                    frontLeft.setPower(leftPower * MAX_SPEED);
                    backLeft.setPower(leftPower * MAX_SPEED);
                    frontRight.setPower(rightPower * MAX_SPEED);
                    backRight.setPower(rightPower * MAX_SPEED);
                    telemetry.addLine("DRIVE_TANK");
                    //telemetry.update();
                    if (leftPower == 0 && rightPower == 0) {
                        command = DRIVE_STATE.DRIVE_STRAFE;
                    }
                    if (gamepad1.b) {
                        command = DRIVE_STATE.SCAN_TO_RIGHT;
                        break;
                    }
                    if (gamepad1.x) {
                        command = DRIVE_STATE.SCAN_TO_LEFT;
                        break;
                    }

                case DRIVE_STRAFE:
                    if (gamepad1.b) {
                        command = DRIVE_STATE.SCAN_TO_RIGHT;
                        break;
                    }
                    if (gamepad1.x) {
                        command = DRIVE_STATE.SCAN_TO_LEFT;
                        break;
                    }
                    if (gamepad1.left_trigger != 0) {
                        double backPower = sameSignSqrt(-gamepad1.left_trigger);
                        double frontPower = sameSignSqrt(gamepad1.left_trigger);
                        frontLeft.setPower(backPower * MAX_SPEED);
                        backRight.setPower(backPower * MAX_SPEED);
                        frontRight.setPower(frontPower * MAX_SPEED);
                        backLeft.setPower(frontPower * MAX_SPEED);

                    } else if (gamepad1.right_trigger != 0) {
                        double frontPower = sameSignSqrt(-gamepad1.right_trigger);
                        double backPower = sameSignSqrt(gamepad1.right_trigger);
                        frontLeft.setPower(backPower * MAX_SPEED);
                        backRight.setPower(backPower * MAX_SPEED);
                        frontRight.setPower(frontPower * MAX_SPEED);
                        backLeft.setPower(frontPower * MAX_SPEED);
                    } else {
                        command = DRIVE_STATE.DRIVE_TANK;
                    }
                    telemetry.addLine("DRIVE_STRAFE");
                    //telemetry.update();

            }
            double slidePower = -gamepad2.left_stick_y;

            if (!slides.isBusy()) {
                if (slides.getCurrentPosition() > BOTTOM_HARDSTOP && slides.getCurrentPosition() < TOP_HARDSTOP) {
                    slides.setPower(slidePower);
                } else if (toggleHardstops) {
                    slides.setPower(slidePower);
                } else if (slides.getCurrentPosition() <= BOTTOM_HARDSTOP && slidePower > 0) {
                    slides.setPower(slidePower);
                } else if (slides.getCurrentPosition() >= TOP_HARDSTOP && slidePower < 0) {
                    slides.setPower(slidePower);
                } else
                    slides.setPower(0);
            }
                switch (slide_position) {
                    case WAIT:
                        if (!slides.barelyBusy()) {
                            if (priorPosition != slideIndex) {
                                priorPosition = slideIndex;
                                slideTimeout = time.time();
                                if (slideIndex == 0) {
                                    slide_position = SLIDE_STATE.BOTTOM;
                                    break;
                                } else if (slideIndex == 1) {
                                    slide_position = SLIDE_STATE.LOW;
                                    break;
                                } else if (slideIndex == 2) {
                                    slide_position = SLIDE_STATE.MEDIUM;
                                    break;
                                } else if (slideIndex == 3) {
                                    slide_position = SLIDE_STATE.HIGH;
                                    break;
                                }
                            }
                        }
                        if (gamepad2.dpad_up) {
                            if (slideIndex < SLIDE_POSITIONS.length - 1) {
                                slideIndex++;
                                if(slides.getCurrentPosition() > SLIDE_POSITIONS[slideIndex]){
                                    slideIndex++;
                                    if(slideIndex != SLIDE_POSITIONS.length - 1 && slides.getCurrentPosition() > SLIDE_POSITIONS[slideIndex+1]){
                                        slideIndex++;
                                    }
                                }
                                break;
                            }
                        }
                        if (gamepad2.dpad_down) {
                            if (slideIndex > 0) {
                                slideIndex--;
                                if(slides.getCurrentPosition() < SLIDE_POSITIONS[slideIndex]){
                                    slideIndex--;
                                    if(slideIndex != 0 && slides.getCurrentPosition() > SLIDE_POSITIONS[slideIndex-1]){
                                        slideIndex--;
                                    }
                                }
                                break;
                            }
                        }
                        if (gamepad2.left_bumper) {
                            slideIndex = 0;
                            slide_position = SLIDE_STATE.BOTTOM;
                            break;
                        }
                        if (gamepad2.right_bumper) {
                            slideIndex = SLIDE_POSITIONS.length - 1;
                            slide_position = SLIDE_STATE.HIGH;
                            break;
                        }
                        break;
                    case BOTTOM:
                        slides.move(SLIDE_POSITIONS[0]);
                        //if (time.seconds() - slideTimeout > SLIDE_TIME) {
                        slide_position = SLIDE_STATE.WAIT;
                        leftServo.setPower(names.leftServoClosedPosition);
                        rightServo.setPower(names.rightServoClosedPosition);
                        //}
                        closedPrior = true;
                        break;
                    case LOW:
                        slides.move(SLIDE_POSITIONS[1]);
                        //if (time.seconds() - slideTimeout > SLIDE_TIME) {
                        slide_position = SLIDE_STATE.WAIT;
                        //}
                        break;
                    case MEDIUM:
                        slides.move(SLIDE_POSITIONS[2]);
                        //if (time.seconds() - slideTimeout > SLIDE_TIME) {
                        slide_position = SLIDE_STATE.WAIT;
                        //}
                        break;
                    case HIGH:
                        slides.move(SLIDE_POSITIONS[SLIDE_POSITIONS.length - 1]);
                        //if (time.seconds() - slideTimeout > SLIDE_TIME) {
                        slide_position = SLIDE_STATE.WAIT;
                        //}
                        break;
                }
                switch (flapperToggle) {
                    case MOVE:
                        if (timer.time(TimeUnit.MILLISECONDS) - currentTime > 500) {
                            flapperToggle = FLAPPER_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        if (gamepad2.x) {
                            if (closedPrior) {
                                flapperToggle = FLAPPER_STATE.MOVE;
                                rightServo.setPower(names.rightServoOpenPosition);
                                leftServo.setPower(names.leftServoOpenPosition);
                                currentTime = timer.time(TimeUnit.MILLISECONDS);
                                closedPrior = false;
                            } else {
                                flapperToggle = FLAPPER_STATE.MOVE;
                                rightServo.setPower(names.rightServoClosedPosition);
                                leftServo.setPower(names.leftServoClosedPosition);
                                currentTime = timer.time(TimeUnit.MILLISECONDS);
                                closedPrior = true;
                            }
                        }
//                    if(slides.getCurrentPosition() < 1650 && slides.getCurrentPosition() > 465 && !closedPrior){
//                        flapperToggle = FLAPPER_STATE.MOVE;
//                        rightServo.setPower(names.rightServoClosedPosition);
//                        leftServo.setPower(names.leftServoClosedPosition);
//                        currentTime = timer.time(TimeUnit.MILLISECONDS);
//                    }
//                    if(slides.getCurrentPosition() < 465 && !closedPrior){
//                        flapperToggle = FLAPPER_STATE.MOVE;
//                        rightServo.setPower(names.rightServoOpenPosition);
//                        leftServo.setPower(names.leftServoOpenPosition);
//                        currentTime = timer.time(TimeUnit.MILLISECONDS);
//                        closedPrior = false;
//                    }
                        break;
                    //1650


                }

                if (gamepad2.right_stick_x != 0) {
                    double rightTarget = rightServo.getPower() - gamepad2.right_stick_x / 200.0;
                    double leftTarget = leftServo.getPower() - gamepad2.right_stick_x / 200.0;
                    if (rightTarget > names.rightHardStopOut && leftTarget < names.leftHardStopOut && rightTarget < names.rightHardStopIn && leftTarget > names.leftHardStopIn) {
                        rightServo.setPower(rightTarget);
                        leftServo.setPower(leftTarget);
                    }
                }
//                if (!toggleHardstops) {
//                    led = LIGHT_STATE.DEFAULT;
//                } else {
//                    led = LIGHT_STATE.OVERRIDE_SLIDES;
//                }

                if (gamepad2.b) {
                    if (toggleHardstops) {
                        if (slideIndex == 0) {
                            BOTTOM_HARDSTOP = slides.getCurrentPosition();
                            SLIDE_POSITIONS[0] = BOTTOM_HARDSTOP;


                        } else if (slideIndex == SLIDE_POSITIONS.length - 1) {
                            TOP_HARDSTOP = slides.getCurrentPosition();
                            SLIDE_POSITIONS[SLIDE_POSITIONS.length - 1] = TOP_HARDSTOP;
                        }
                        toggleHardstops = false;
                        while (gamepad2.b) {
                            telemetry.addLine("Waiting for User to Release B");
                            telemetry.update();
                        }
                        telemetry.clear();
                        telemetry.update();
                    } else {
                        toggleHardstops = true;
                        while (gamepad2.b) {
                            telemetry.addLine("waiting for User to Release B");
                            telemetry.update();
                        }
                        telemetry.clear();
                        telemetry.update();
                    }
                }

                if (slides.isBusy()) {
                    slides.update();
                }


                telemetry.addData("Left Target Power", leftTgtPower);
                telemetry.addData("Right Target Power", rightTgtPower);
                telemetry.addData("Front Right Motor Power", frontRight.getPower());
                telemetry.addData("Front Left Motor Power", frontLeft.getPower());
                telemetry.addData("Back Right Motor Power", backRight.getPower());
                telemetry.addData("Back Left Motor Power", backLeft.getPower());
                telemetry.addData("Slide Positions", slides.getCurrentPos());
                telemetry.addData("Slide Target", slides.targetPos);
                telemetry.addData("Slide State", slides.isBusy());
                telemetry.addData("Slide Has Reached:", slides.getReached());
                telemetry.addData("Status", "Running");
                telemetry.addData("Voltage", slides.getBatteryVoltage() + "V");
                telemetry.addData("Gamepad2 Right x Power", gamepad2.right_stick_x);
                telemetry.update();
            }

            rightServo.setPower(names.rightServoClosedPosition);
            leftServo.setPower(names.leftServoClosedPosition);
        }



    public double closerToV2(double v1, double v2, double v3){
        double diff1 = Math.abs(v1-v2);
        double diff2 = Math.abs(v2-v3);
        if (diff1 > diff2){
            return v1;
        }
        return v3;
    }

    public double sameSignSqrt(double number){
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
}