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
    public static double GAIN = 30;
    //1540 1008 86
    enum SLIDE_STATE {
            IDLE,
            GOING_UP,
            GOING_DOWN
    }
    SLIDE_STATE slideState = SLIDE_STATE.IDLE;
    public boolean auto = false;
    private ElapsedTime runtime = new ElapsedTime();
    public int positions[] = {-5107, -1520, 644, 2800};

    @Override
    public void runOpMode() {
        testingMotor = hardwareMap.get(DcMotor.class, "slides");//change name to servo that is being tested.
        testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testingMotor.setDirection(DcMotor.Direction.FORWARD);
        testingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        color = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        telemetry.addLine(testingMotor.getCurrentPosition() + "");
        telemetry.update();
        NormalizedRGBA colors;
        waitForStart();
        double power = 0;
        while (opModeIsActive() && !isStopRequested()) {
            switch (slideState){
                case IDLE:
                    auto = false;
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
                    if (gamepad1.y || gamepad2.y){
                        slideState = SLIDE_STATE.GOING_UP;
                        color.setGain((float) GAIN);
                        testingMotor.setPower(.3);
                        auto = true;
                    }
                    if (gamepad1.x || gamepad2.x){
                        slideState = SLIDE_STATE.GOING_DOWN;
                        color.setGain((float) GAIN);
                        colors = color.getNormalizedColors();
                        testingMotor.setPower(-.3);
                        auto = true;

                    }

                case GOING_UP:
                    telemetry.addLine("UP");
                    colors = color.getNormalizedColors();
                    if (colors.green > .29 && colors.red > .58){
                        slideState = SLIDE_STATE.IDLE;
                        testingMotor.setPower(0);
                    }
                    if (gamepad1.b || gamepad2.b){
                        slideState = SLIDE_STATE.IDLE;
                        testingMotor.setPower(0);
                    }
                    break;
                case GOING_DOWN:
                    telemetry.addLine("DOWN");
                    colors = color.getNormalizedColors();
                    if (colors.green > .29 && colors.red > .58){
                        colors = color.getNormalizedColors();
                        testingMotor.setPower(-.5);
                        sleep(500);
                        testingMotor.setPower(0);
                        slideState = SLIDE_STATE.IDLE;
                    }
                    if (gamepad1.b || gamepad2.b){
                        slideState = SLIDE_STATE.IDLE;
                        testingMotor.setPower(0);
                    }
                    break;
            }

            telemetry.addLine(testingMotor.getCurrentPosition() + "");

            if(gamepad1.a || gamepad2.a){
                testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (!auto) {
                testingMotor.setPower(power);
            }
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

    public void encoderIntake(double target, double timeoutS) {
        double currentTime = System.currentTimeMillis();
        double slidesPosition = testingMotor.getCurrentPosition();
        if (slidesPosition > target) {
            testingMotor.setPower(-.5);
            while (slidesPosition > target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = testingMotor.getCurrentPosition();
            }
            testingMotor.setPower(0);
        } else if (slidesPosition < target) {
            testingMotor.setPower(.5);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = testingMotor.getCurrentPosition();
            }
            testingMotor.setPower(0);
        }
    }
}
