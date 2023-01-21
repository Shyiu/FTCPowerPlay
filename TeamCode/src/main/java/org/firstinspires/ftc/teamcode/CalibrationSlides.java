package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="Calibrate Slide to 0 Position", group="Testing_Servo")
public class CalibrationSlides extends LinearOpMode {
    DcMotor testingMotor;
    NormalizedColorSensor color;
    public static double GAIN = 15;
    //1540 1008 86
    private ElapsedTime runtime = new ElapsedTime();
    public int positions[] = {-5107, -1520, 644, 2800};

    @Override
    public void runOpMode() {
        testingMotor = hardwareMap.get(DcMotor.class, "slides");//change name to servo that is being tested.
        testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testingMotor.setDirection(DcMotor.Direction.REVERSE);
        testingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        color = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        telemetry.addLine(testingMotor.getCurrentPosition() + "");
        telemetry.update();
        NormalizedRGBA colors;
        waitForStart();
        double power = 0;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y){
                color.setGain((float) GAIN);
                colors = color.getNormalizedColors();
                while (colors.green < .45 && colors.red < .45){
                    testingMotor.setPower(.3);
                    colors = color.getNormalizedColors();
                }
            }
            telemetry.addLine(testingMotor.getCurrentPosition() + "");
            if (Math.abs(gamepad1.left_stick_y) > 0){
                power = -gamepad1.left_stick_y/3.0;
            }
            else if (Math.abs(gamepad2.left_stick_y) > 0){
                power = -gamepad2.left_stick_y/3.0;
            }
            else if(gamepad1.right_bumper || gamepad2.right_bumper){
                power = 1;
            }
            else if(gamepad1.left_bumper || gamepad2.left_bumper){
                power = -1;
            }
            else{
                power = 0;
            }
            if(gamepad1.a || gamepad2.a){
                testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad1.x || gamepad2.x){
                color.setGain((float) GAIN);
                colors = color.getNormalizedColors();
                while (colors.green < .45 && colors.red < .45){
                    testingMotor.setPower(-.3);
                    colors = color.getNormalizedColors();
                }
                testingMotor.setPower(-.5);
                sleep(500);
                testingMotor.setPower(0);
            }
            testingMotor.setPower(power);
            colors = color.getNormalizedColors();
            color.setGain((float)GAIN);
            telemetry.addLine("Gamepad (1 or 2) Controls:\nLeft Stick Y Moves Motor\na resets the encoder position\ny brings the slides up to the color sensor\n x brings the slides down to be below color sensor");
            telemetry.addData("Color Blue", colors.blue);
            telemetry.addData("Color Red", colors.red);
            telemetry.addData("Color Green", colors.green);
            telemetry.addData("Slide Position", testingMotor.getCurrentPosition());
            telemetry.addData("Slide power", testingMotor.getPower());
            telemetry.addData("power", -gamepad1.left_stick_y/3.0);
            telemetry.update();
//            testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(int ticks) {
        int newLeftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = ticks;


            testingMotor.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            testingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            testingMotor.setPower(-.5);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (testingMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newLeftTarget);
                telemetry.addData("Currently at", " at %7d",
                        testingMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            testingMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
