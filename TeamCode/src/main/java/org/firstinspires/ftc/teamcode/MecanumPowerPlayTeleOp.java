package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@TeleOp(name = "Mecanum Power Play Teleop", group = "Tele-Op")
public class MecanumPowerPlayTeleOp extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slides;
    protected DcMotor tape;
    protected DcMotorSimple flapper;
    protected DistanceSensor distance;
    final boolean autoSlides = true;

    double leftTgtPower = 0, rightTgtPower = 0;
    double y,x,rx,y2;
    double denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
    public PowerplayBot names = new PowerplayBot();
    //60 is encoder position
    //Slide Related Variables
    final double TOP_HARDSTOP = 93.5;
    final double BOTTOM_HARDSTOP = 27.8;//Actually supposed to be 0
    final int STARTING_POS = 719;
    final double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 41, 66, TOP_HARDSTOP};
    int slideIndex = 0;
    double slidesPosition = 0;
    final double SLIDE_POWER = .9;

    //Flap related Variables
    final double flapUp = .379;
    final double flapDown = .77;

    public MecanumPowerPlayTeleOp() throws Exception
    {
        Logging.setup();
        Logging.log("Starting Tele-Op Logging");
    }
    @Override
    public void mainInit() {
        slides = hardwareMap.get(DcMotor.class, names.slides);
        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);
        distance = hardwareMap.get(DistanceSensor.class, names.distance);
        tape = hardwareMap.get(DcMotor.class, names.tape);

        tape.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slide position", distance.getDistance(DistanceUnit.CM));


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
                //Function of Game Controller 1
                y = -gamepad1.left_stick_y; // Remember, this is reversed!
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
                y2 = -gamepad2.left_stick_y;
                Servo.Direction direction = Servo.Direction.FORWARD;





                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;




                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);


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

                double distanceCM = distance.getDistance(DistanceUnit.CM);
                if (distanceCM > BOTTOM_HARDSTOP && distanceCM < TOP_HARDSTOP) {
                    slides.setPower(slidePower);
                }
                else if (distanceCM <= BOTTOM_HARDSTOP && slidePower > 0) {
                    slides.setPower(slidePower);
                }
                else if (distanceCM >= TOP_HARDSTOP && slidePower < 0) {
                    slides.setPower(slidePower);
                }
                else
                    slides.setPower(0);

                tape.setPower(tapePower);

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
            if (distance.getDistance(DistanceUnit.CM) > target) {
                slides.setPower(-SLIDE_POWER);
                while (distance.getDistance(DistanceUnit.CM) > target && getRuntime() - currentTime < 5) {
                    telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));
                    telemetry.addData("Target Position", target);
                    telemetry.update();
                }
                slides.setPower(0);
            } else if (distance.getDistance(DistanceUnit.CM) < target) {
                slides.setPower(SLIDE_POWER);
                while (distance.getDistance(DistanceUnit.CM) < target && getRuntime() - currentTime < 5) {
                    telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));
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
        telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));





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
