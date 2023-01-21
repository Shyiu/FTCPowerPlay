/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "Mecanum Power Play Auto Try 2", group = "Skystone",preselectTeleOp = "Mecanum Power Play Teleop")
public class MecanumPowerPlayAutoTry2 extends LinearOpMode
{
    public static double SLIDE_POWER = .5;
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;
    public static NormalizedColorSensor color_sensor;
    public static int INCHES_TO_HIGH_JUNCTION_BEFORE_TURN = 43;
    public static int DEGREES_TO_HIGH_JUNCTION = 28;
    public static int INCHES_TO_HIGH_JUNCTION_AFTER_TURN = 6;
    public static boolean TRY_STRAFE_ONE = false;
    public static boolean TRY_STRAFE_TWO = false;
    public static boolean TURN = false;
    final int ID_TAG_OF_INTEREST = 0;
    final int ID_TAG_OF_INTEREST_2 = 4;
    final int ID_TAG_OF_INTEREST_3 = 7;
    final double[] SERVO_POS = {.58,1,.22,0};
    final public PowerplayBot names = new PowerplayBot();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    final double TAGSIZE = 0.05;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_ENCODER_REV = 8192;
    static final double DEADWHEEL_DIAMETER_INCHES = 1.378;
    //35:45 = 3.72 ft/s 40:40 = 2.88 ft/s 45:35 = 2.26 ft/s  (0.25 initially)
    //we need to figure out exact gear ratio without trial and error
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // No External Gearing.
    // Diameter - 4.0 for mecanum, 4.0 for other
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_DEADWHEEL_INCH = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION) / (DEADWHEEL_DIAMETER_INCHES * Math.PI);
    public static double     DRIVE_SPEED_FR             = 0.5;
    public static double     DRIVE_SPEED_FL             = 0.5;
    public static double     DRIVE_SPEED_BR             = 0.5;
    public static double     DRIVE_SPEED_BL             = 0.5;

    public static int TIME_MS = 1000;
    static double     TURN_SPEED              = 0.5;
    static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;
    static final DcMotor.RunMode STOP = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    static final DcMotor.RunMode RUN = DcMotor.RunMode.RUN_USING_ENCODER;
    int parking_zone = 2;
    final double flapUp = .57;
    final double flapDown = .77;
    AprilTagDetection tagOfInterest = null;
    public static int distanceFor360Turn = 1000;
    //Motor Setup


    // Reverses the direction of the left motors, to allow a positive motor power to equal
    // forwards and a negative motor power to equal backwards
    DcMotor frontRight, frontLeft, backRight, backLeft, slides;
    DcMotorSimple flapper;

    //Creates the motor with the given name and direction. Sets correct, modes, and zero power behaviour*/
    public DcMotor initMotor(String motorName, DcMotor.Direction direction){
        DcMotor motorVariable = hardwareMap.get(DcMotor.class, motorName);
        motorVariable.setDirection(direction);
        motorVariable.setMode(STOP);
        motorVariable.setMode(RUN);
        motorVariable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motorVariable;
    }

    /**Creates the motor with the given name. Sets correct modes, direction, and zero power behaviour*/
    public DcMotor initMotor(String motorName){
        DcMotor motorVariable = hardwareMap.get(DcMotor.class, motorName);
        motorVariable.setDirection(FORWARD);
        motorVariable.setMode(STOP);
        motorVariable.setMode(RUN);
        motorVariable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motorVariable;
    }

    /**Main Code*/
    @Override
    public void runOpMode()
    {

        //Initializing the motors
        frontRight = initMotor(names.fr, REVERSE);
        frontLeft = initMotor(names.fl);
        backRight = initMotor(names.br, REVERSE);
        backLeft = initMotor(names.bl);
        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);
        slides = hardwareMap.get(DcMotor.class, names.slides);
        slides.setDirection(REVERSE);


        //Initializing the IMU and PID


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(0,power);
//        pidDrive.setInputRange(-90,90);
//        pidDrive.enable();
      /**  color.setGain((float) 15);
        slides.setPower(SLIDE_POWER);
        NormalizedRGBA colors = color.getNormalizedColors();
        double currentTime = getRuntime();
        while (colors.green < .45 && colors.red < .45){
            if( getRuntime() - currentTime < 2){
                slides.setPower(0);
                break;
            }
            colors = color.getNormalizedColors();

        }
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setPower(0);
        encoderIntake(-400, 2);**/

      //  encoderIntake(-4000, 1);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, names.camera), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(TAGSIZE, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        //This loop is initializing the camera and trying to identify an AprilTag using AprilTagPipeline
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST || tag.id == ID_TAG_OF_INTEREST_2 || tag.id == ID_TAG_OF_INTEREST_3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addLine("Information for Tag of Interest");

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            if(tagOfInterest.id == 0){
                parking_zone = 1;
            }
            if(tagOfInterest.id == 4){
                parking_zone = 3;
            }
            if(tagOfInterest.id == 7){
                parking_zone = 2;
            }
            tagToTelemetry(tagOfInterest,parking_zone);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");

            telemetry.update();
        }

        if(tagOfInterest == null)
        {
//            normalDrive();
            encoderDrive(.3, 24, 24, 2.3);
//            if (TRY_STRAFE_ONE) {
//                strafe(.6, 24, 3, "right");
//            }
//            if(TRY_STRAFE_TWO) {
//                strafe(.6, 24, 3, "left");
//            }
//            if(TURN){
//                turnDegrees(.8,90);
//            }

        }
        else
        {

            if (parking_zone == 2){
//normalDrive();
//                resetSlides();
//                encoderDrive(DRIVE_SPEED, 31, 31,5);

            }


            if (parking_zone == 1){
//                normalDrive();
//                resetSlides();
//                encoderDrive(DRIVE_SPEED, 28, 28,5);
//                flapper.setPower(.3);
//
//                sleep(250);
//                flapper.setPower(flapDown);
//                sleep(250);
//                turnDegrees(TURN_SPEED, 90);
//
//                sleep(250);
//                encoderDrive(DRIVE_SPEED, 15, 15,5);
//                sleep(250);
//                flapper.setPower(flapDown);

            }
            if (parking_zone == 3){
//                resetSlides();
//                encoderDrive(DRIVE_SPEED, 29, 29,5);
//                flapper.setPower(.3);
//
//                sleep(250);
//                flapper.setPower(flapDown);
//                sleep(250);
//
//                turnDegrees(TURN_SPEED, -85);
//                sleep(250);
//                encoderDrive(DRIVE_SPEED, 18, 18,5);
//                sleep(250);
//                flapper.setPower(flapDown);
            }
        }
    }
    /** Adds that the camera has detected the April_Tag in telemetry*/
    void tagToTelemetry(AprilTagDetection detection, int parking_zone)
    {
        telemetry.addData("Current Parking Zone: ", parking_zone);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addData("Current Parking Zone: ", parking_zone);

    }

    /** Adds that the camera has seen but not currently seeing the April_Tag in telemetry*/
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addData("Current Parking Zone: ", parking_zone);

//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }

    /** Move the slides using SLIDE_POWER until they reach target or timeoutS seconds has passed */
    public void encoderIntake(double target, double timeoutS) {
        double currentTime = getRuntime();
        double slidesPosition = slides.getCurrentPosition();
        if (slidesPosition > target) {
            slides.setPower(-SLIDE_POWER);
            while (slidesPosition > target && getRuntime() - currentTime < timeoutS) {
                telemetry.addData("Slide Position", slides.getCurrentPosition());
                telemetry.addData("Target Position", target);
                telemetry.update();
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        } else if (slidesPosition < target) {
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && getRuntime() - currentTime < timeoutS) {
                telemetry.addData("Slide Position", slides.getCurrentPosition());
                telemetry.addData("Target Position", target);
                telemetry.update();
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
    }


    /**Turns on all the motors at speed speed and runs them until one motor reachs its target or timeoutS seconds have passed*/
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {


        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            frontLeft.setPower(DRIVE_SPEED_FL);
            frontRight.setPower(DRIVE_SPEED_FR);
            backLeft.setPower(DRIVE_SPEED_BL);
            backRight.setPower(DRIVE_SPEED_BR);
            sleep(TIME_MS);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);


            sleep(250);   // optional pause after each move.
        }

    }



}
