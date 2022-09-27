package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ManualMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Motors controlled by Game Controller 1
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Motors controlled by Game Controller 2
        DcMotor intakeControl = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor slideControl = hardwareMap.dcMotor.get("sliderMotor");

        slideControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Function of Game Controller 1
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //Function of Game Controller 2
            double intakePower = signedSquareRoot((signedSquare(-gamepad2.right_stick_x) + signedSquare(-gamepad2.right_stick_y)));
            double sliderPower = -gamepad2.left_stick_y;

            intakeControl.setPower(intakePower);
            slideControl.setPower(sliderPower);
            telemetry.addData("position", slideControl.getCurrentPosition());

            telemetry.update();
        }
    }

    public double signedSquare(double num) {
        if (num > 0) {
            return (num*num);
        }
        else {
            return (-1*num*num);
        }
    }

    public double signedSquareRoot (double num) {
        if (num > 0) {
            return (Math.pow(num, 0.5));
        } else {
            return (-Math.pow(-num, 0.5));
        }
    }
}