package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@TeleOp(name = "PowerPlay Tele Op", group = "Tele-Op")
public class PowerplayTeleOp extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
    protected Servo flapper;
    public PowerplayBot names = new PowerplayBot();

    //Slide Related Variables
    final int TOP_HARDSTOP = 3788;
    final int BOTTOM_HARDSTOP = 0;//Actually supposed to be 0

    final double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 1556, 2660, TOP_HARDSTOP};
    int slideIndex = 0;
    double slidesPosition = 0;
    final double SLIDE_POWER = .75;

   //Flap related Variables
    final double flapUp = .33;
    final double flapDown = .43;

    public PowerplayTeleOp() throws Exception
    {
        Logging.setup();
        Logging.log("Starting Tele-Op Logging");
    }
    @Override
    public void mainInit() {
        slides = hardwareMap.get(DcMotor.class, names.slides);
        flapper = hardwareMap.get(Servo.class, names.intake);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesPosition = slides.getCurrentPosition();


        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slide position", slides.getCurrentPosition());

        flapper.setPosition(flapUp);

        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);


        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


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
                telemetry.addData("Slide Position", slides.getCurrentPosition());





                // Status: Running
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }));
        registerThread(new TaskThread(new TaskThread.Actions(){
            @Override
            public void loop(){
                Logging.log(Integer.toString(slides.getCurrentPosition()));
            }
        }));
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                double slidePower = -gamepad2.left_stick_y;
                if (slides.getCurrentPosition() > BOTTOM_HARDSTOP && slidePower < 0) {
                    slides.setPower(slidePower);
                    Logging.log("Moving Slides Down");
                }
                else
                    slides.setPower(0);

                if (slides.getCurrentPosition() < TOP_HARDSTOP && slidePower > 0) {
                    slides.setPower(slidePower);
                    Logging.log("Moving Slides Up");
                }
                else
                    slides.setPower(0);

                if (gamepad2.dpad_up){
                    if (slideIndex < SLIDE_POSITIONS.length - 1){
                        slideIndex++;
                        Logging.log("Moving Slides Up");
                    }
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.dpad_down){
                    if (slideIndex > 0){
                        slideIndex--;
                        Logging.log("Moving Slides Down");
                    }
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                }
                if (gamepad2.left_bumper){
                    slideIndex = 0;
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                    Logging.log("Bringing Slides all the way down");
                }
                if(gamepad2.right_bumper){
                    slideIndex = SLIDE_POSITIONS.length - 1;
                    moveSlides(SLIDE_POSITIONS[slideIndex]);
                    Logging.log("Bringing Slides all the way up");
                }
                if(gamepad2.x){
                    if(flapper.getPosition() == flapUp){
                        flapper.setPosition(flapDown);
                        Logging.log("Lowering Flap");
                        while(gamepad2.x){
                            telemetry.addLine("Waiting for User to Release");
                            telemetry.update();
                        }
                    }
                    else if(flapper.getPosition() == flapDown){
                        flapper.setPosition(flapUp);
                        Logging.log("Raising Flap");
                        while(gamepad2.x){
                            telemetry.addLine("Waiting for User to Release");
                            telemetry.update();
                        }
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
        double currentTime = getRuntime();
        if (slidesPosition > target){
            slides.setPower(-SLIDE_POWER);
            while (slidesPosition > target  && getRuntime() - currentTime < 5){
                telemetry.addData("Slide Position",slides.getCurrentPosition());
                telemetry.addData("Target Position",target);
                telemetry.update();
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
        else if (slidesPosition < target){
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && getRuntime() - currentTime < 5){
                telemetry.addData("Slide Position",slides.getCurrentPosition());
                telemetry.addData("Target Position",target);
                telemetry.update();
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
    }
    @Override
    public void mainLoop() {

    }

}
