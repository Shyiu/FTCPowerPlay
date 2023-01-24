package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@Config
@TeleOp(name = "Mecanum Power Play Teleop", group = "Tele-Op")
public class MecanumPowerPlayTeleOp extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
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
    @Override
    public void mainInit() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, names.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        slides = hardwareMap.get(DcMotor.class, names.slides);
        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        color = hardwareMap.get(NormalizedColorSensor.class, names.color);



        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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


        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                switch (command){
//                    case ROBOT_CENTRIC:
//                        //The loop method should contain loop code
//                        //Function of Game Controller 1
//                        y = -gamepad1.left_stick_y; // Remember, this is reversed!
//                        x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//                        rx = gamepad1.right_stick_x;
//                        y2 = -gamepad2.left_stick_y;
//
//
//
//
//                        // Denominator is the largest motor power (absolute value) or 1
//                        // This ensures all the powers maintain the same ratio, but only when
//                        // at least one is out of the range [-1, 1]
//                        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                        frontLeftPower = (y + x + rx) / denominator;
//                        backLeftPower = (y - x + rx) / denominator;
//                        frontRightPower = (y - x - rx) / denominator;
//                        backRightPower = (y + x - rx) / denominator;
//                        frontLeft.setPower(frontLeftPower);
//                        backLeft.setPower(backLeftPower);
//                        frontRight.setPower(frontRightPower);
//                        backRight.setPower(backRightPower);
//                        telemetry.addLine("ROBOT CENTRIC DRIVING");
//                        telemetry.update();


                    case DRIVE_TANK:
                        double leftPower = -gamepad1.left_stick_y;
                        double rightPower = -gamepad1.right_stick_y;
                        frontLeft.setPower(leftPower);
                        backLeft.setPower(leftPower);
                        frontRight.setPower(rightPower);
                        backRight.setPower(rightPower);
                        telemetry.addLine("DRIVE_TANK");
                        telemetry.update();
                        if(leftPower == 0 && rightPower == 0) {
                            command = DRIVE_STATE.DRIVE_STRAFE;
                        }

                    case DRIVE_STRAFE:
                        if (gamepad1.left_trigger != 0){
                            double backPower = -gamepad1.left_trigger;
                            double frontPower = gamepad1.left_trigger;
                            frontLeft.setPower(backPower);
                            backRight.setPower(backPower);
                            frontRight.setPower(frontPower);
                            backLeft.setPower(frontPower);

                        }
                        else if (gamepad1.right_trigger != 0){
                            double frontPower = -gamepad1.right_trigger;
                            double backPower = gamepad1.right_trigger;
                            frontLeft.setPower(backPower);
                            backRight.setPower(backPower);
                            frontRight.setPower(frontPower);
                            backLeft.setPower(frontPower);
                        }
                        else {
                            command = DRIVE_STATE.DRIVE_TANK;
                        }
                        telemetry.addLine("DRIVE_STRAFE");
                        telemetry.update();

//                    case FIELD_CENTRIC:
//                        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//                        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//                        double rx = gamepad1.right_stick_x;
//                        double speed_reduction_factor = 1;
//                        // Read inverse IMU heading, as the IMU heading is CW positive
//                        double botHeading = -imu.getAngularOrientation().firstAngle;
//
//                        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
//                        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
//
//                        // Denominator is the largest motor power (absolute value) or 1
//                        // This ensures all the powers maintain the same ratio, but only when
//                        // at least one is out of the range [-1, 1]
//                        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                        double frontLeftPower = (rotY + rotX + rx) / denominator;
//                        double backLeftPower = (rotY - rotX + rx) / denominator;
//                        double frontRightPower = (rotY - rotX - rx) / denominator;
//                        double backRightPower = (rotY + rotX - rx) / denominator;
//
//                        frontLeft.setPower(frontLeftPower * speed_reduction_factor);
//                        backLeft.setPower(backLeftPower * speed_reduction_factor);
//                        frontRight.setPower(frontRightPower * speed_reduction_factor);
//                        backRight.setPower(backRightPower * speed_reduction_factor);
//                        telemetry.addLine("FIELD_CENTRIC");
//                        telemetry.update();

                }








                // Makes the Driver Hub output the message
                // Left Target Power: A float from [-1,1]
                // Right Target Power: A float from [-1,1]
                // The current power of each motor


            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
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
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.dpad_down){
                    if (slideIndex > 0){
                        slideIndex--;
                    }
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.left_bumper){
                    slideIndex = 0;
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                }
                if(gamepad2.right_bumper){
                    slideIndex = SLIDE_POSITIONS.length - 1;
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
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
                if(gamepad2.x){
                    flapper.setPower(closerToV2(flapUp, flapper.getPower(), flapDown));
                    while(gamepad2.x){
                        telemetry.addLine("Waiting for User to Release X");
                        telemetry.update();
                    }
                    telemetry.clear();
                    telemetry.update();

                }
            }
        }));
    }
    public void moveSlides(double target){
        double currentTime = getRuntime();

        if(autoSlides) {
            if (slides.getCurrentPosition() > target) {
                slides.setPower(-SLIDE_POWER);

                while (slides.getCurrentPosition() > target && getRuntime() - currentTime < 5) {
                    if (getBatteryVoltage() < 10.0){
                        break;
                    }
                    telemetry.addData("Slide Position", slides.getCurrentPosition());
                    telemetry.addData("Target Position", target);
                    telemetry.update();
                }
                slides.setPower(0);
            } else if (slides.getCurrentPosition() < target) {
                slides.setPower(SLIDE_POWER);
                while (slides.getCurrentPosition() < target && getRuntime() - currentTime < 5) {
                    if (getBatteryVoltage() < 10.0){
                        break;
                    }
                    telemetry.addData("Slide Position", slides.getCurrentPosition());
                    telemetry.addData("Target Position", target);
                    telemetry.update();
                }
                slides.setPower(0);
            }
        }
    }
    @Override
    public void mainLoop() {
        telemetry.addData("Left Target Power", leftTgtPower);
        telemetry.addData("Right Target Power", rightTgtPower);
        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());
        telemetry.addData("Slide Position", slides.getCurrentPosition());





        // Status: Running
        telemetry.addData("Status", "Running");
        telemetry.update();
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
}
