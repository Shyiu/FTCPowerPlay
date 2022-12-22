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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "PowerPlay Pushbot Auto", group = "Skystone")
public class PowerPlayAuto extends LinearOpMode
{
    private static double SLIDE_POWER = .5;
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;
    public static NormalizedColorSensor color_sensor;
    public static int INCHES_TO_HIGH_JUNCTION_BEFORE_TURN = 43;
    public static int DEGREES_TO_HIGH_JUNCTION = 28;
    public static int INCHES_TO_HIGH_JUNCTION_AFTER_TURN = 6;
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
    //35:45 = 3.72 ft/s 40:40 = 2.88 ft/s 45:35 = 2.26 ft/s  (0.25 initially)
    //we need to figure out exact gear ratio without trial and error
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // No External Gearing.
    // Diameter - 4.0 for mecanum, 4.0 for other
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static double     DRIVE_SPEED             = 0.5;
    static double     TURN_SPEED              = 0.5;
    static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;
    static final DcMotor.RunMode STOP = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    static final DcMotor.RunMode RUN = DcMotor.RunMode.RUN_USING_ENCODER;
    int parking_zone = 2;
    final double flapUp = .57;
    final double flapDown = .77;
    AprilTagDetection tagOfInterest = null;

    //Motor Setup


    // Reverses the direction of the left motors, to allow a positive motor power to equal
    // forwards and a negative motor power to equal backwards
    DcMotor frontRight, frontLeft, backRight, backLeft, slides;
    DcMotorSimple flapper;
    DistanceSensor distance;
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .5, correction, rotation;
    PIDController           pidRotate, pidDrive;

    /**Creates the motor with the given name and direction. Sets correct, modes, and zero power behaviour*/
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
        frontRight = initMotor(names.fr);
        frontLeft = initMotor(names.fl, REVERSE);
        backRight = initMotor(names.br);
        backLeft = initMotor(names.bl, REVERSE);

        flapper = hardwareMap.get(DcMotorSimple.class, names.intake);
        slides = hardwareMap.get(DcMotor.class, names.slides);
        distance = hardwareMap.get(DistanceSensor.class, names.distance);


        //Initializing the IMU and PID
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, names.imu);

        imu.initialize(parameters);

        pidRotate = new PIDController(0.003,0.000003,0);

        pidDrive = new PIDController(.15,0,0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0,power);
        pidDrive.setInputRange(-90,90);
        pidDrive.enable();

        encoderIntake(32, 1);
        flapper.setPower(.88);

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
            normalDrive();

        }
        else
        {

            if (parking_zone == 2){
//normalDrive();
                resetSlides();
                encoderDrive(DRIVE_SPEED, 31, 31,5);

            }


            if (parking_zone == 1){
//                normalDrive();
                resetSlides();
                encoderDrive(DRIVE_SPEED, 28, 28,5);
                flapper.setPower(.3);

                sleep(250);
                flapper.setPower(flapDown);
                sleep(250);
                turnDegrees(TURN_SPEED, 90);

                sleep(250);
                encoderDrive(DRIVE_SPEED, 15, 15,5);
                sleep(250);
                flapper.setPower(flapDown);

            }
            if (parking_zone == 3){
                resetSlides();
                encoderDrive(DRIVE_SPEED, 29, 29,5);
                flapper.setPower(.3);

                sleep(250);
                flapper.setPower(flapDown);
                sleep(250);

                turnDegrees(TURN_SPEED, -85);
                sleep(250);
                encoderDrive(DRIVE_SPEED, 18, 18,5);
                sleep(250);
                flapper.setPower(flapDown);
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
    public void encoderIntake(double target, double timeoutS){
            double currentTime = getRuntime();
            double slidesPosition = distance.getDistance(DistanceUnit.CM);
                if (slidesPosition > target) {
                    slides.setPower(-SLIDE_POWER);
                    while (slidesPosition > target && getRuntime() - currentTime < 5) {
                        telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));
                        telemetry.addData("Target Position", target);
                        telemetry.update();
                        slidesPosition = distance.getDistance(DistanceUnit.CM);
                    }
                    slides.setPower(0);
                } else if (slidesPosition < target) {
                    slides.setPower(SLIDE_POWER);
                    while (slidesPosition < target && getRuntime() - currentTime < 5) {
                        telemetry.addData("Slide Position", distance.getDistance(DistanceUnit.CM));
                        telemetry.addData("Target Position", target);
                        telemetry.update();
                        slidesPosition = distance.getDistance(DistanceUnit.CM);
                    }
                    slides.setPower(0);
                }
    }

    /**PID FUNCTION*/
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**PID FUNCTION*/
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**PID FUNCTION*/
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**Turn at power speed until the PID detects a change by degrees degrees*/
    private void turnDegrees(double power, int degrees)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();


        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359,degrees);


        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint. Started with 100 but robot did not
        // slow and overshot the turn. Increasing I slowed the end of the turn and completed
        // the turn in a timely manner

        // the turn in a timely manner
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {

                backRight.setPower(-power);
                frontRight.setPower(-power);
                frontLeft.setPower(power);
                backLeft.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.

                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.

                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**Turns on all the motors at speed speed and runs them until one motor reachs its target or timeoutS seconds have passed*/
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active

        if (opModeIsActive()) {
            frontLeft.setMode(STOP);
            frontRight.setMode(STOP);
            backLeft.setMode(STOP);
            backRight.setMode(STOP);

            frontLeft.setMode(RUN);
            frontRight.setMode(RUN);
            backLeft.setMode(RUN);
            backRight.setMode(RUN);

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            // reset the timeout time and start motion.


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            Boolean mode1 = mode(backRight, newBackRightTarget);
            Boolean mode2 = mode(backLeft, newBackLeftTarget);
            Boolean mode3 = mode(frontRight, newRightTarget);
            Boolean mode4 = mode(frontLeft, newLeftTarget);

            runtime.reset();
            double frontLeftSpeed = speedConversion(mode1, speed);
            double frontRightSpeed = speedConversion(mode2, speed);
            double backLeftSpeed = speedConversion(mode3, speed);
            double backRightSpeed = speedConversion(mode4, speed);
            frontLeft.setPower(speedConversion(mode1, speed));
            frontRight.setPower(speedConversion(mode2, speed));
            backLeft.setPower(speedConversion(mode3, speed));
            backRight.setPower(speedConversion(mode4, speed));

            while (opModeIsActive() &&
                    (isBusy(backRight, newBackRightTarget, mode1) && isBusy(backLeft, newBackLeftTarget, mode2) && isBusy(frontRight, newRightTarget, mode3) && isBusy(frontLeft, newLeftTarget, mode4))) {
                correction = pidDrive.performPID(getAngle());

                frontLeft.setPower(Math.max(.3,frontLeftSpeed - correction));
                frontRight.setPower(Math.max(.3,frontRightSpeed + correction));
                backLeft.setPower(Math.max(.3,backLeftSpeed - correction));
                backRight.setPower(Math.max(.3,backRightSpeed + correction));

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Back Currently at",  " at %7d :%7d",

                        backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.addData("Inverse Current Position", frontLeft.getCurrentPosition() * -1);

                telemetry.addData("Correction Value:", correction);
                telemetry.update();

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION

            sleep(250);   // optional pause after each move.
        }

    }

    /**Takes the robot from the starting position on the right side and drives it to the high junction, turns, scores, turns back, drives back to the
     * 1st tile of the second parking zone
     */
    public void normalDrive(){

        encoderIntake(36, 1);
        flapper.setPower(flapUp);
        encoderIntake(27.3,1);
        flapper.setPower(flapDown);
        encoderIntake(46,2);
        sleep(250);
        encoderDrive(DRIVE_SPEED, INCHES_TO_HIGH_JUNCTION_BEFORE_TURN, INCHES_TO_HIGH_JUNCTION_BEFORE_TURN, 5);
        sleep(250);
        turnDegrees(TURN_SPEED, DEGREES_TO_HIGH_JUNCTION);
        sleep(250);
        encoderIntake(95.5,2);
        sleep(250);
        encoderDrive(DRIVE_SPEED, INCHES_TO_HIGH_JUNCTION_AFTER_TURN, INCHES_TO_HIGH_JUNCTION_AFTER_TURN, 5);
        sleep(250);
        flapper.setPower(flapUp);
        sleep(250);






    }

    /** Flaps the flapper (not used in the parking zone code*/
    public void resetSlides(){
        flapper.setPower(flapUp);
        sleep(250);
//        encoderIntake(0,5);
        flapper.setPower(flapDown);
        sleep(250);

//        encoderIntake(3356,5);
    }

    /** Returns a boolean that depends on the motor.currentPosition() compared to position*/
    public boolean mode(DcMotor motor, int position){
        return -motor.getCurrentPosition() < position;
    }

    /** Function that compares the motor position to their targets*/
    public boolean isBusy(DcMotor motor, int position, boolean mode){
        int motorPos = -motor.getCurrentPosition();
        Logging.log(motor.getDeviceName() + "'s current target is " + position + "and current position is " + motorPos);
        if (mode){
            return motorPos < position;
        }
        else{
            return motorPos > position;
        }

    }

    /** Returns a the input positive or negative based on the direction the robot needs to move*/
    public double speedConversion(Boolean mode, double speed){
        if (mode){
            return (speed);
        }
        else{
            return (-speed);
        }

    }
    public void colorDrive(){
        if (opModeIsActive()) {
            frontLeft.setMode(STOP);
            frontRight.setMode(STOP);
            backLeft.setMode(STOP);
            backRight.setMode(STOP);

            frontLeft.setMode(RUN);
            frontRight.setMode(RUN);
            backLeft.setMode(RUN);
            backRight.setMode(RUN);

            // Determine new target position, and pass to motor controlle


            // reset the timeout time and start motion.


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            runtime.reset();
            double frontLeftSpeed = .5;
            double frontRightSpeed = .5;
            double backLeftSpeed = .5;
            double backRightSpeed = .5;
            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);
            NormalizedRGBA colors = color_sensor.getNormalizedColors();
            color_sensor.setGain((float)6.25);
            //6.25 .4
            while (opModeIsActive() && colors.blue < .022) {
                colors = color_sensor.getNormalizedColors();
                correction = pidDrive.performPID(getAngle());

                frontLeft.setPower(Math.max(.3,frontLeftSpeed - correction));
                frontRight.setPower(Math.max(.3,frontRightSpeed + correction));
                backLeft.setPower(Math.max(.3,backLeftSpeed - correction));
                backRight.setPower(Math.max(.3,backRightSpeed + correction));

                // Display it for the driver.
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Back Currently at",  " at %7d :%7d",

                        backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.addData("Inverse Current Position", frontLeft.getCurrentPosition() * -1);

                telemetry.addData("Blue Value:", colors.blue);

                telemetry.update();

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION

            sleep(250);   // optional pause after each move.
        }

    }
}
