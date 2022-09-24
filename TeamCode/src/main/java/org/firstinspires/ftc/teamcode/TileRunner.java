package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Takes the current y-position of both the left and right joy-sticks
            // The highest position of a joystick is equal to -1, and the bottommost position of the
            // joystick is equal to 1
            double leftTgtPower = -this.gamepad1.left_stick_y;
            double rightTgtPower = -this.gamepad1.right_stick_y;

            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed
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
    }
}
