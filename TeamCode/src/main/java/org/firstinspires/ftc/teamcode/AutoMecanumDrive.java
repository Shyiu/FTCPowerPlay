package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp
public class AutoMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        /**
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
        DcMotor duckControl = hardwareMap.dcMotor.get("duckMotor");

        slideControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
**/
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Status", "Failed");

                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });





        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
/**
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

            if (gamepad2.dpad_down) {
                slideControl.setTargetPosition(FLOOR);
                slideControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideControl.setPower(1);

                while(opModeIsActive() && slideControl.isBusy()){
                    telemetry.addData("Path1", "Running");
                    telemetry.update();
                }

                slideControl.setPower(0);
                slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else if (gamepad2.dpad_left) {
                slideControl.setTargetPosition(FIRST_LEVEL);
                slideControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideControl.setPower(1);

                while(opModeIsActive() && slideControl.isBusy()){
                    telemetry.addData("Ticks", slideControl.getCurrentPosition());
                    telemetry.update();
                }

                slideControl.setPower(0);
                slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else if (gamepad2.dpad_up) {
                slideControl.setTargetPosition(SECOND_LEVEL);
                slideControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while(opModeIsActive() && slideControl.isBusy()){
                    telemetry.addData("Path1", "Running");
                    telemetry.update();
                }

                slideControl.setPower(0);
                slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else if (gamepad2.dpad_right){
                slideControl.setTargetPosition(THIRD_LEVEL);
                slideControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideControl.setPower(1);

                while(opModeIsActive() && slideControl.isBusy()){
                    telemetry.addData("Path1", "Running");
                    telemetry.update();
                }

                slideControl.setPower(0);
                slideControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

            if (gamepad2.x) {
                duckControl.setPower(1);
            } else if (gamepad2.b) {
                duckControl.setPower(-1);
            } else {
                duckControl.setPower(0);
            }

            intakeControl.setPower(intakePower);
            //slideControl.setPower(sliderPower);
            telemetry.addData("right", gamepad2.dpad_right);
            telemetry.addData("up", gamepad2.dpad_up);
            telemetry.addData("left", gamepad2.dpad_left);
            telemetry.addData("down", gamepad2.dpad_down);

            telemetry.update();
 */
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