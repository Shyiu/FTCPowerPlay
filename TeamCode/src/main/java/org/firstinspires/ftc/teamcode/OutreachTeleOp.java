package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
// Tells the robot that this is TeleOp code (hello)
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

// LinearOpMode means that code is read sequentially
public class OutreachTeleOp extends LinearOpMode {

    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor slide;
    protected DcMotorSimple clutch;
    protected DcMotorSimple test;

    @Override
    public void runOpMode() {

        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        slide = hardwareMap.get(DcMotor.class, "sliderMotor");
        clutch = hardwareMap.get(DcMotorSimple.class, "clutchMotor");
        test = hardwareMap.get(DcMotorSimple.class, "testMotor");
        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Takes the current y-position of both the left and right joy-sticks
            // The highest position of a joystick is equal to -1, and the bottommost position of the
            // joystick is equal to 1
            double leftTgtPower = -this.gamepad1.left_stick_y;
            double rightTgtPower = this.gamepad1.right_stick_y;
            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed
            frontLeft.setPower(leftTgtPower);
            backLeft.setPower(leftTgtPower);
            frontRight.setPower(rightTgtPower);
            backRight.setPower(rightTgtPower);
            //slide power control
            if(gamepad1.dpad_up) {
                slide.setPower(1);
            } else if(gamepad1.dpad_down) {
                slide.setPower(-1);
            } else{
                slide.setPower(0);
            }

            //Servo power control
            double servoPower = 1.2;
            if(gamepad1.b){
                clutch.setDirection(DcMotorSimple.Direction.FORWARD);
                clutch.setPower(1);
            }else if(gamepad1.a){
                clutch.setDirection(DcMotorSimple.Direction.FORWARD);
                clutch.setPower(.42);
            }

            //testing servo power control
            if (gamepad1.y) {
                test.setPower(1);
            }
            else if(gamepad1.x){
                test.setPower(.7);
            }
            // Makes the Driver Hub output the message
            // Left Target Power: A float from [-1,1]
            // Right Target Power: A float from [-1,1]
            // The current power of each motor
            telemetry.addData("Pink Piece Servo Direction", clutch.getDirection());
            telemetry.addData("Pink Piece Servo Power", clutch.getPower());

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
    }
}