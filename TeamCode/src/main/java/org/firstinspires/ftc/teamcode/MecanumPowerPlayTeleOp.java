package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.Lift;
import org.firstinspires.ftc.teamcode.obselete.PowerplayBot;

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
    protected DcMotor tape;
    protected DcMotorSimple flapper;
    public static double MAX_SPEED = 1;
    protected NormalizedColorSensor color;
    final double FLAPPER_TIME = 0.5; //Amount of time till flappers timeout
    public enum DRIVE_STATE{
        DRIVE_TANK,
        DRIVE_STRAFE,
//        FIELD_CENTRIC,
//        ROBOT_CENTRIC,
    }
    public enum SLIDE_STATE{
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
        WAIT
    }

    SLIDE_STATE slide_position = SLIDE_STATE.WAIT;
    //Between field centric, straightforward and my weird version
    double leftTgtPower = 0, rightTgtPower = 0;

    public PowerplayBot names = new PowerplayBot();
    //60 is encoder position
    //Slide Related Variables
    double TOP_HARDSTOP = 7115;
    double BOTTOM_HARDSTOP = -339;//Actually supposed to be 0
    double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 2761, 5020, TOP_HARDSTOP};
    int slideIndex = 0;
    boolean toggleHardstops = false;
    //Flap related Variables
    final double flapUp = .379;
    final double flapDown = .77;

    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

    public void runOpMode() throws InterruptedException{
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, names.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        slides = new Lift(hardwareMap, 72, 16, 8);
        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);

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


        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();
        while(!isStopRequested() && opModeIsActive()) {

            switch (command) {
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

                case DRIVE_STRAFE:
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
                    if(!slides.barelyBusy()) {
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
                        if (gamepad2.dpad_up) {
                            if (slideIndex < SLIDE_POSITIONS.length - 1) {
                                slideIndex++;
                                break;
                            }
                        }
                        if (gamepad2.dpad_down) {
                            if (slideIndex > 0) {
                                slideIndex--;
                                break;
                            }
                        }
                        if (gamepad2.left_bumper) {
                            slideIndex = 0;
                            break;
                        }
                        if (gamepad2.right_bumper) {
                            slideIndex = SLIDE_POSITIONS.length - 1;
                            break;
                        }
                    }
                    break;
                case BOTTOM:
                    slides.move(SLIDE_POSITIONS[0]);
                    //if (time.seconds() - slideTimeout > SLIDE_TIME) {
                    slide_position = SLIDE_STATE.WAIT;
                    //}
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
            if(gamepad2.dpad_left || gamepad2.dpad_right){
                slides.move(240);
            }
            if (gamepad2.b) {
                if (toggleHardstops) {
                    if (slideIndex == 0) {
                        double difference = slides.getCurrentPosition() - BOTTOM_HARDSTOP;
                        BOTTOM_HARDSTOP = slides.getCurrentPosition();
                        SLIDE_POSITIONS[0] = BOTTOM_HARDSTOP;
                        SLIDE_POSITIONS[1] += difference;
                        SLIDE_POSITIONS[2] += difference;

                    } else if (slideIndex == SLIDE_POSITIONS.length - 1) {
                        double difference = TOP_HARDSTOP - slides.getCurrentPosition();
                        TOP_HARDSTOP = slides.getCurrentPosition();
                        SLIDE_POSITIONS[SLIDE_POSITIONS.length - 1] = TOP_HARDSTOP;
                        SLIDE_POSITIONS[1] -= difference;
                        SLIDE_POSITIONS[2] -= difference;
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

            if (time.seconds() - flapperTimeout > FLAPPER_TIME) {
                if (gamepad2.x) {
                    flapperTimeout = time.time();
                    flapper.setPower(closerToV2(flapUp, flapper.getPower(), flapDown));
                    telemetry.addLine("Waiting for User to Release X");
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
            telemetry.update();
        }

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