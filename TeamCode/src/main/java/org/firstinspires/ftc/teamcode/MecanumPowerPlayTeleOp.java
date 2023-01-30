package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.Lift;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@Config
@TeleOp(name = "Mecanum Power Play Teleop", group = "Tele-Op")
public class MecanumPowerPlayTeleOp extends LinearOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected Lift slides;
    protected DcMotor tape;
    protected DcMotorSimple flapper;
    protected NormalizedColorSensor color;
    final boolean autoSlides = true;
    public enum DRIVE_STATE{
        DRIVE_TANK,
        DRIVE_STRAFE,
//        FIELD_CENTRIC,
//        ROBOT_CENTRIC,
    }
    //Between field centric, straightforward and my weird version
    double leftTgtPower = 0, rightTgtPower = 0;
    double y,x,rx,y2;
    double denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
    public PowerplayBot names = new PowerplayBot();
    //60 is encoder position
    //Slide Related Variables
    double TOP_HARDSTOP = 6933;
    double BOTTOM_HARDSTOP = -870;//Actually supposed to be 0
    final int STARTING_POS = 719;
    double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 2646, 4984, TOP_HARDSTOP};
    int slideIndex = 0;
    double slidesPosition = 0;
    final double SLIDE_POWER = .9;
    boolean toggleHardstops = false;
    //Flap related Variables
    final double flapUp = .379;
    final double flapDown = .77;

    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;
    public MecanumPowerPlayTeleOp() throws Exception
    {
        Logging.setup();
        Logging.log("Starting Tele-Op Logging");
    }
    public void runOpMode() throws InterruptedException{
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, names.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        slides = new Lift(hardwareMap, 1, 2, 3);
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
        while(!isStopRequested() && opModeIsActive()){

            switch (command) {
                case DRIVE_TANK:
                    double leftPower = sameSignSqrt(-gamepad1.left_stick_y);
                    double rightPower = sameSignSqrt(-gamepad1.right_stick_y);
                    frontLeft.setPower(leftPower);
                    backLeft.setPower(leftPower);
                    frontRight.setPower(rightPower);
                    backRight.setPower(rightPower);
                    telemetry.addLine("DRIVE_TANK");
                    telemetry.update();
                    if (leftPower == 0 && rightPower == 0) {
                        command = DRIVE_STATE.DRIVE_STRAFE;
                    }

                case DRIVE_STRAFE:
                    if (gamepad1.left_trigger != 0) {
                        double backPower = sameSignSqrt(-gamepad1.left_trigger);
                        double frontPower = sameSignSqrt(gamepad1.left_trigger);
                        frontLeft.setPower(backPower);
                        backRight.setPower(backPower);
                        frontRight.setPower(frontPower);
                        backLeft.setPower(frontPower);

                    } else if (gamepad1.right_trigger != 0) {
                        double frontPower = sameSignSqrt(-gamepad1.right_trigger);
                        double backPower = sameSignSqrt(gamepad1.right_trigger);
                        frontLeft.setPower(backPower);
                        backRight.setPower(backPower);
                        frontRight.setPower(frontPower);
                        backLeft.setPower(frontPower);
                    } else {
                        command = DRIVE_STATE.DRIVE_TANK;
                    }
                    telemetry.addLine("DRIVE_STRAFE");
                    telemetry.update();

            }
                double slidePower = -gamepad2.left_stick_y;

                if (slides.getCurrentPosition() > BOTTOM_HARDSTOP && slides.getCurrentPosition() < TOP_HARDSTOP) {
                    slides.setPower(slidePower);
                }
                else if(toggleHardstops){
                    slides.setPower(slidePower);
                }
                else if (slides.getCurrentPosition() <= BOTTOM_HARDSTOP && slidePower > 0) {
                    slides.setPower(slidePower);
                }
                else if (slides.getCurrentPosition() >= TOP_HARDSTOP && slidePower < 0) {
                    slides.setPower(slidePower);
                }
                else
                    slides.setPower(0);


                if (gamepad2.dpad_up){
                    if (slideIndex < SLIDE_POSITIONS.length - 1){
                        slideIndex++;
                    }
                    slides.move(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.dpad_down){
                    if (slideIndex > 0){
                        slideIndex--;
                    }
                    slides.move(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.left_bumper){
                    slideIndex = 0;
                    slides.move(SLIDE_POSITIONS[slideIndex]);
                }
                if(gamepad2.right_bumper){
                    slideIndex = SLIDE_POSITIONS.length - 1;
                    slides.move(SLIDE_POSITIONS[slideIndex]);
                }
                if(gamepad2.b){
                    if (toggleHardstops){
                        if (slideIndex == 0){
                            double difference = slides.getCurrentPosition() - BOTTOM_HARDSTOP;
                            BOTTOM_HARDSTOP = slides.getCurrentPosition();
                            SLIDE_POSITIONS[0] = BOTTOM_HARDSTOP;
                            SLIDE_POSITIONS[1] += difference;
                            SLIDE_POSITIONS[2] += difference;

                        }
                        else if (slideIndex == SLIDE_POSITIONS.length - 1){
                            double difference = TOP_HARDSTOP - slides.getCurrentPosition();
                            TOP_HARDSTOP = slides.getCurrentPosition();
                            SLIDE_POSITIONS[SLIDE_POSITIONS.length - 1] = TOP_HARDSTOP;
                            SLIDE_POSITIONS[1] -= difference;
                            SLIDE_POSITIONS[2] -= difference;
                        }
                        toggleHardstops = false;
                        while(gamepad2.b){
                            telemetry.addLine("Waiting for User to Release B");
                            telemetry.update();
                        }
                        telemetry.clear();
                        telemetry.update();
                    }
                    else{
                        toggleHardstops = true;
                        while(gamepad2.b){
                            telemetry.addLine("waiting for User to Release B");
                            telemetry.update();
                        }
                        telemetry.clear();
                        telemetry.update();
                    }
                }
                if(gamepad2.x)
                {
                    flapper.setPower(closerToV2(flapUp, flapper.getPower(), flapDown));
                    while (gamepad2.x) {
                        telemetry.addLine("Waiting for User to Release X");
                        telemetry.update();
                    }
                    telemetry.clear();
                    telemetry.update();
                }
        slides.update();
        telemetry.addData("Left Target Power", leftTgtPower);
        telemetry.addData("Right Target Power", rightTgtPower);
        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());

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
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double sameSignSqrt(double number){
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
}
