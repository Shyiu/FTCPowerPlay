package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Skystone Tele Op", group = "Tele-Op")
public class SkystoneTeleOp extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
//    protected Servo armJoint2;
    protected Servo flapper;


    //Slide Related Variables
    final double[] SLIDE_POSITIONS = {0,100,200,300,400,500,600,700,800};
    int slideIndex = 0;
    double slidesPosition = 0;
    final double SLIDE_POWER = .1;
    final int TOP_HARDSTOP = 2000;
    final int BOTTOM_HARDSTOP = 100;


    @Override
    public void mainInit() {
        slides = hardwareMap.get(DcMotor.class, "slides");
//        armJoint2 = hardwareMap.get(Servo.class, "joint_servo");
//        claw = hardwareMap.get(Servo.class, "claw_servo");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double flapUp = .78;
        double flapDown = 1;

        slidesPosition = slides.getCurrentPosition();

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moveSlides(BOTTOM_HARDSTOP);

        flapper.setPosition(flapDown);

        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");



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
                //The loop method should contain loop code
                double leftTgtPower = -gamepad1.left_stick_y;
                double rightTgtPower = -gamepad1.right_stick_y;
                frontLeft.setPower(leftTgtPower);
                backLeft.setPower(leftTgtPower);
                frontRight.setPower(rightTgtPower);
                backRight.setPower(rightTgtPower);


                // Makes the Driver Hub output the message
                // Left Target Power: A float from [-1,1]
                // Right Target Power: A float from [-1,1]
                // The current power of each motor
                telemetry.addData("Left Target Power", leftTgtPower);
                telemetry.addData("Right Target Power", rightTgtPower);
                telemetry.addData("Front Right Motor Power", frontRight.getPower());
                telemetry.addData("Front Left Motor Power", frontLeft.getPower());
                telemetry.addData("Back Right Motor Power", backRight.getPower());
                telemetry.addData("Back Left Motor Power", backLeft.getPower());




                // Status: Running
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }));
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                double slidePower = -gamepad2.left_stick_y;
                slides.setPower(slidePower);

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
                if(gamepad2.x){
                    if(flapper.getPosition() == flapUp){
                        flapper.setPosition(flapDown);
                    }
                    if(flapper.getPosition() == flapDown){
                        flapper.setPosition(flapUp);
                    }
                    while(gamepad2.x){
                        telemetry.addLine("Waiting for User to Release");
                        telemetry.update();
                    }
                    telemetry.clear();
                    telemetry.update();

                }
            }
        }));
    }
    public static double negSqrt(double value){
        if(value >= 0){
            return Math.sqrt(value);
        }
        else{
            return -1 * Math.sqrt((-1*value));
        }
    }
    public void moveSlides(double target){
        int currentPos = slides.getCurrentPosition();
        if (currentPos > target){
            slides.setPower(-SLIDE_POWER);
            while (currentPos > target){
                telemetry.addData("Slide Position",slides.getCurrentPosition());
                telemetry.addData("Target Position",target);
                telemetry.update();
            }
            slides.setPower(0);
        }
        if (currentPos < target){
            slides.setPower(SLIDE_POWER);
            while (currentPos < target){
                telemetry.addData("Slide Position",slides.getCurrentPosition());
                telemetry.addData("Target Position",target);
                telemetry.update();
            }
            slides.setPower(0);
        }
    }
    @Override
    public void mainLoop() {

    }

}
