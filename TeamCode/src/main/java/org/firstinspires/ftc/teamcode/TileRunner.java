package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TileRunner extends LinearOpMode {

    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;

    @Override
    public void runOpMode() {

        // Pulls the motors from the robot configuration so that they can be manipulated
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        DcMotor motorSlide = hardwareMap.dcMotor.get("motorSlide");

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo claw = hardwareMap.get(Servo.class, "claw_servo");
        //double clawOpen = 1;
        //double clawClose = -.5;




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Takes the current y-position of both the left and right joy-sticks
            // The highest position of a joystick is equal to -1, and the bottommost position of the
            // joystick is equal to 1
            double leftTgtPower = -this.gamepad1.left_stick_y;
            double rightTgtPower = -this.gamepad1.right_stick_y;

            double slidePower = -this.gamepad2.left_stick_y;

            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed
            frontLeft.setPower(leftTgtPower);
            backLeft.setPower(leftTgtPower);
            frontRight.setPower(rightTgtPower);
            backRight.setPower(rightTgtPower);
            motorSlide.setPower(slidePower);

            // Makes the Driver Hub output the message
            // Left Target Power: A float from [-1,1]
            // Right Target Power: A float from [-1,1]
            // The current power of each motor
            telemetry.addData("Left Target Power", leftTgtPower);
            telemetry.addData("Right Target Power", rightTgtPower);
            telemetry.addData("Slide Motor Power", slidePower);
            //Servo.Direction direction = Servo.Direction.FORWARD;

            //claw.setDirection(direction);
            //if (gamepad2.y) {
            //    claw.setPosition(clawOpen);
            //}
            //else if(gamepad2.x){
              //  claw.setPosition(clawClose);
            //}

            //telemetry.addData("Servo Power" , claw.getPosition());
            //telemetry.update();

            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Slide Motor Power", motorSlide.getPower());


            // Status: Running
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
