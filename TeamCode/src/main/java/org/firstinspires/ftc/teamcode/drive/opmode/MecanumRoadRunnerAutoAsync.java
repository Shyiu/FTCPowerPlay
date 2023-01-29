package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.PowerplayBot;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * TODO: Calibrate the PID so the slides stop as close to the target as possible. The target is outputed to telemetry. Use Dashboard to edit the values quickly.
 * TODO: Being cycling off of the cone stack. In order to do use lineToHeading or something like that when making the spline path so the robot drives straight and turns to the correct heading. Move the slides at the same time.
 * TODO: Slides.control moves the slides while pausing the code
 * TODO: Slides.move will move the slides async as long as slides.update is in a loop
 * */

@Config
@Autonomous(name = "Mecanum Roadrunner Async")
public class MecanumRoadRunnerAutoAsync extends LinearOpMode {
    public static int parking_zone = 2;
    public static double zone3X = 20;
    final double flapUp = .379;
    final double flapDown = .77;
    public static double zone1Y = 31;
    public static double zone1X = 17;

    enum DRIVE_STATE {
        PRELOAD,
        FIRST_TRAJ,
        SEC_TRAJ,
        PARK,
        IDLE
    }

    DRIVE_STATE state = DRIVE_STATE.PRELOAD;
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;
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

    HashMap<Integer, Integer> tagDict = new HashMap<>();
    final int ID_TAG_OF_INTEREST = 0;
    final int ID_TAG_OF_INTEREST_2 = 4;
    final int ID_TAG_OF_INTEREST_3 = 7;
    public static double P = 30;
    public static double I = 6;
    public static double D = 0;

    // UNITS ARE METERS
    final double TAGSIZE = 0.05;
    private ElapsedTime runtime = new ElapsedTime();


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift slides = new Lift(hardwareMap, P, I, D);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Intake flapper = new Intake(hardwareMap);
        Pose2d startPose = new Pose2d(33, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence firstDrive = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(15)
                .lineToLinearHeading(new Pose2d(11.5, -34.2, Math.toRadians(121.25)))
                .build();

        Trajectory toJunction = drive.trajectoryBuilder(firstDrive.end())
                .forward(5)
                .build();
        Trajectory toStack = drive.trajectoryBuilder(toJunction.end())
                .splineToSplineHeading(new Pose2d(57, -12, Math.toRadians(0)), Math.toRadians(-15))
                .build();
        Trajectory scoreStack = drive.trajectoryBuilder(toStack.end())
                .lineToLinearHeading(new Pose2d(33.1, -8.6, Math.toRadians(128)))
                .build();
        Trajectory toStackAfter = drive.trajectoryBuilder(scoreStack.end())
                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(0)))
                .build();


                Trajectory zone3Strafe = drive.trajectoryBuilder(toStackAfter.end())
                        .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(90)))
                .build();

        Trajectory zone1Strafe = drive.trajectoryBuilder(toStackAfter.end())
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(90)))
                .build();

        Trajectory zone2Strafe = drive.trajectoryBuilder(toStackAfter.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                .build();


        tagDict.put(0, 1);
        tagDict.put(7, 2);
        tagDict.put(4, 3);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, names.camera), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(TAGSIZE, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        slides.init();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

//        This loop is initializing the camera and trying to identify an AprilTag using AprilTagPipeline
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST || tag.id == ID_TAG_OF_INTEREST_2 || tag.id == ID_TAG_OF_INTEREST_3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addLine("Information for Tag of Interest");
                    telemetry.addData("Parking Zone: ", tagDict.get(tagOfInterest.id));
                    parking_zone = tagDict.get(tagOfInterest.id);

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        telemetry.addData("Parking Zone: ", tagDict.get(tagOfInterest.id));
                        parking_zone = tagDict.get(tagOfInterest.id);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    telemetry.addData("Parking Zone: ", tagDict.get(tagOfInterest.id));
                    parking_zone = tagDict.get(tagOfInterest.id);
                }

            }

            telemetry.update();
            sleep(20);
        }
        state = DRIVE_STATE.PRELOAD;
        slides.control(500, 3, 1);
        flapper.move(flapUp);
        sleep(750);
        slides.control(-600, 2, 1);
        flapper.move(flapDown);
        sleep(500);
        slides.control(500, 3, 1);
//        drive.followTrajectorySequenceAsync(firstDrive);
        slides.move(4000);
        while (opModeIsActive() && !isStopRequested()) {
//            switch (state) {
//                case PRELOAD:
//                    telemetry.addLine("PRELOAD");
//                    if (!drive.isBusy() && !slides.isBusy()) {
//                        state = DRIVE_STATE.FIRST_TRAJ;
//                        drive.followTrajectoryAsync(toJunction);
//
//                    }
//                    break;
//                case FIRST_TRAJ:
//                    telemetry.addLine("FIRST_TRAJ");
//
//                    if(!drive.isBusy() && !slides.isBusy()) {
//                        flapper.move(flapUp);
//                        sleep(1000);
//                        state = DRIVE_STATE.SEC_TRAJ;
//                    }
//                    break;
//                case SEC_TRAJ:
//                    telemetry.addLine("SEC_TRAJ");
//
//
//                    if(!slides.isBusy() && !drive.isBusy()) {
//                        if (parking_zone == 1) {
//                            drive.followTrajectoryAsync(zone1Strafe);
//                        } else if (parking_zone == 3) {
//                            drive.followTrajectoryAsync(zone3Strafe);
//                        } else {
//                            drive.followTrajectoryAsync(zone2Strafe);
//                        }
//                        state = DRIVE_STATE.PARK;
//                    }
//                    break;
//                case PARK:
//                    telemetry.addLine("PARK");
//
//                    if (!drive.isBusy() && !slides.isBusy()) {
//                        slides.move(0);
//                        state = DRIVE_STATE.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    telemetry.addLine("IDLE");
//
//                    break;
//            }
//            drive.update();
            slides.update();
            telemetry.addData("pos", slides.getCurrentPos());
            telemetry.update();
        }
    }
}


