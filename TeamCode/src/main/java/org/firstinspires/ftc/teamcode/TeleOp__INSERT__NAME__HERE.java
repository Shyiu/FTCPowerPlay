//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class TeleOp__INSERT__NAME__HERE {
//    DcMotor frontLeft = hardwareMap.dcMotor.get(front_LEFT_NAME);
//    DcMotor frontRight = hardwareMap.dcMotor.get(front_RIGHT_NAME);
//    DcMotor backRight = hardwareMap.dcMotor.get(back_LEFT_NAME);
//    DcMotor backLeft = hardwareMap.dcMotor.get(back_RIGHT_NAME);
//
//    DcMotor intakeMotor = hardwareMap.dcMotor.get(intake_MOTOR_NAME);
//    DcMotor slideControl = hardwareMap.dcMotor.get(slide_MOTOR_NAME);
//
//    waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//        //Function of Game Controller 1
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
//
//        //Function of Game Controller 2
//        double intakePower = signedSquareRoot((signedSquare(-gamepad2.right_stick_x) + signedSquare(-gamepad2.right_stick_y)));
//        double sliderPower = -gamepad2.left_stick_y;
//
//        intakeMotor.setPower(intakePower);
//        slideControl.setPower(sliderPower);
//        telemetry.addData("position", slideControl.getCurrentPosition());
//
//        telemetry.update();
//    }
//    public double signedSquare(double num) {
//        if (num > 0) {
//            return (num*num);
//        }
//        else {
//            return (-1*num*num);
//        }
//    }
//
//    public double signedSquareRoot (double num) {
//        if (num > 0) {
//            return (Math.pow(num, 0.5));
//        } else {
//            return (-Math.pow(-num, 0.5));
//        }
//    }
//}
//
//}
