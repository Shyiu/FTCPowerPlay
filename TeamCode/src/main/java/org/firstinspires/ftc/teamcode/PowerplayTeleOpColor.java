package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@Disabled
@TeleOp(name = "PowerPlay Tele Op With Color", group = "Tele-Op")
public class PowerplayTeleOpColor extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
    protected DcMotor tape;
    protected DcMotorSimple flapper;
    protected NormalizedColorSensor color;
    final boolean autoSlides = true;

    double leftTgtPower = 0, rightTgtPower = 0;
    public PowerplayBot names = new PowerplayBot();
    //60 is encoder position
    //Slide Related Variables
    public static double TOP_HARDSTOP = 2800;
    public static double BOTTOM_HARDSTOP = -5107;
    final int STARTING_POS = 719;
    public static double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, -1520, 644, TOP_HARDSTOP};
    public static boolean toggleHardstops = false;
    int slideIndex = 0;
    double slidesPosition = 0;
    final double SLIDE_POWER = .9;

   //Flap related Variables
    final double flapUp = .379;
    final double flapDown = .77;

    public PowerplayTeleOpColor() throws Exception
    {
        Logging.setup();
        Logging.log("Starting Tele-Op Logging");
    }
    @Override
    public void mainInit() {
        slides = hardwareMap.get(DcMotor.class, names.slides);
        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);
        color = hardwareMap.get(NormalizedColorSensor.class, names.color);
        tape = hardwareMap.get(DcMotor.class, names.tape);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                leftTgtPower = -gamepad1.left_stick_y;
                rightTgtPower = -gamepad1.right_stick_y;
                frontLeft.setPower(leftTgtPower);
                backLeft.setPower(leftTgtPower);
                frontRight.setPower(rightTgtPower);
                backRight.setPower(rightTgtPower);


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
                double tapePower = gamepad2.left_trigger;

                tape.setPower(tapePower);
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
                    telemetry.addData("Slide Position", slides.getCurrentPosition());
                    telemetry.addData("Target Position", target);
                    telemetry.update();
                }
                slides.setPower(0);
            } else if (slides.getCurrentPosition() < target) {
                slides.setPower(SLIDE_POWER);
                while (slides.getCurrentPosition() < target && getRuntime() - currentTime < 5) {
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
//        telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));





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
}
