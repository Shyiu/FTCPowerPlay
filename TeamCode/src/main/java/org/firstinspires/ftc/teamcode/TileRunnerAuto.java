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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Tilerunner Auto", group = "Tilerunner")
public class TileRunnerAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;

    final int ID_TAG_OF_INTEREST = 0;
    final int ID_TAG_OF_INTEREST_2 = 4;
    final int ID_TAG_OF_INTEREST_3 = 7;
    final double[] SERVO_POS = {.58,1,.22,0};


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
    static final double     DRIVE_GEAR_REDUCTION    = 35/45.0;     // No External Gearing.
    // Diameter - 4.0 for mecanum, 4.0 for other
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.03;
    static final double     TURN_SPEED              = 0.5;
    static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;

    int parking_zone = 2;
    double clawOpen = .78;
    double clawClose = 1;
    AprilTagDetection tagOfInterest = null;

    //Motor Setup


    // Reverses the direction of the left motors, to allow a positive motor power to equal
    // forwards and a negative motor power to equal backwards
    DcMotor frontRight, frontLeft, backRight, backLeft, armJoint1;
    Servo armJoint2, claw;




    public DcMotor initMotor(String motorName, DcMotor.Direction direction){
        DcMotor motorVariable = hardwareMap.get(DcMotor.class, motorName);
        motorVariable.setDirection(direction);
        motorVariable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVariable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVariable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motorVariable;
    }
    public DcMotor initMotor(String motorName){
        DcMotor motorVariable = hardwareMap.get(DcMotor.class, motorName);
        motorVariable.setDirection(FORWARD);
        motorVariable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVariable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVariable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motorVariable;
    }
    @Override
    public void runOpMode()
    {

        frontRight = initMotor("frontRight", REVERSE);
        frontLeft = initMotor("frontLeft");
        backRight = initMotor("backRight", REVERSE);
        backLeft = initMotor("backLeft");
        armJoint1 = initMotor("armJoint1");

//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//
//
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//       armJoint1  = hardwareMap.get(DcMotor.class, "armJoint1");
//        armJoint1.setDirection(DcMotor.Direction.FORWARD);
        armJoint2 = hardwareMap.get(Servo.class, "joint_servo");
        claw = hardwareMap.get(Servo.class, "claw_servo");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */

            //Guessing Parking Zone 2 If Tag Not Found

            encoderDrive(DRIVE_SPEED, 16, 16, 5);
            turnDegreesLeft(DRIVE_SPEED, 30);
            armJoint2.setPosition(1);
            encoderIntake(DRIVE_SPEED, 1540, 2);
            armJoint2.setPosition(0);
            claw.setPosition(clawOpen);
            armJoint2.setPosition(1);
            claw.setPosition(clawClose);
            encoderIntake(DRIVE_SPEED, 0, 2);
            turnDegreesRight(DRIVE_SPEED, 30);
        }
        else
        {
            if (parking_zone == 2){
                encoderDrive(DRIVE_SPEED, 12, 12, 2);//

            }


            if (parking_zone == 1){
                encoderDrive(DRIVE_SPEED, 12, 12, 2);

                encoderDrive(DRIVE_SPEED,-8,8,2);
                encoderDrive(DRIVE_SPEED, 12, 12, 2);

            }
            if (parking_zone == 3){
                encoderDrive(DRIVE_SPEED, 12, 12, 2);
                encoderDrive(DRIVE_SPEED,8,-8,2);
                encoderDrive(DRIVE_SPEED, 12, 12, 2);
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    void tagToTelemetry(AprilTagDetection detection, int parking_zone)
    {
        telemetry.addData("Current Parking Zone: ", parking_zone);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addData("Current Parking Zone: ", parking_zone);

    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }
    public void turnDegreesRight(double speed, double degrees){
        //Make these numbers the amount for a full 360 degree turn. CW direction in inches.
        double fullTurnLeft = 64;//16 inches side length times 4
        double fullTurnRight = -64;


        double turnAmount = degrees/360.0;//Convert degrees into a fraction.

        fullTurnRight *= turnAmount;
        fullTurnLeft *= turnAmount;
        encoderDrive(speed, fullTurnLeft, fullTurnRight, 5);//Drives using encoder drive.
    }
    public void turnDegreesLeft(double speed, double degrees){
        //Make these numbers the amount for a full 360 degree turn. CCW direction in inches.
        double fullTurnLeft = -64;//16 inches side length times 4
        double fullTurnRight = 64;


        double turnAmount = degrees/360.0;//Convert degrees into a fraction.

        fullTurnRight *= turnAmount;
        fullTurnLeft *= turnAmount;
        encoderDrive(speed, fullTurnLeft, fullTurnRight, 5);//Drives using encoder drive.
    }
    public void encoderIntake(double speed, int ticks, double timeoutS){

        if(opModeIsActive()){
            armJoint1.setTargetPosition((ticks));
            armJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double currentTime = getRuntime();
            armJoint1.setPower(Math.abs(speed));
            while (opModeIsActive() && getRuntime() - currentTime < timeoutS && isBusy(armJoint1, ticks)){
                telemetry.addData("Position", armJoint1.getCurrentPosition());
            }
            armJoint1.setPower(0);
            armJoint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (isBusy(backRight, -newBackRightTarget) && isBusy(backLeft, -newBackLeftTarget) && isBusy(frontRight, -newRightTarget) && isBusy(frontLeft, -newLeftTarget))) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Direction BLeft", backLeft.getDirection());
                telemetry.addData("Direction FLeft", frontLeft.getDirection());

                telemetry.addData("Direction bRight", backRight.getDirection());
                telemetry.addData("Direction fRight", frontRight.getDirection());

                telemetry.update();

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }
    public boolean isBusy(DcMotor motor, int position){
        return Math.abs(motor.getCurrentPosition()) < Math.abs(position);
    }
}