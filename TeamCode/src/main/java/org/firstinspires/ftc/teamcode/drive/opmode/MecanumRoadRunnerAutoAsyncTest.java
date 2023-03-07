package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.obselete.PowerplayBot;
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
@Autonomous(name = "Mecanum Roadrunner Async TEST")
public class MecanumRoadRunnerAutoAsyncTest extends LinearOpMode {
    public static int parking_zone = 2;
    public static double zone3X = 20;
    final double flapDown = .77;
    public static double zone1Y = 31;
    public static double zone1X = 17;
    public double pauseTime = 0;
    enum DRIVE_STATE {
        PRELOAD,
        FIRST_TRAJ,
        SEC_TRAJ,
        PARK,
        STACK_FIRST,
        SCORE_STACK,
        toJunction,
        SCORE_STACK2,
        SCORE_STACK3,
        CYCLE,
        STACK,
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
    public static double P = 72;
    public static double I = 16;
    public static double D = 8;

    // UNITS ARE METERS
    final double TAGSIZE = 0.05;
    private ElapsedTime runtime = new ElapsedTime();


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift slides = new Lift(hardwareMap, P, I, D, 3);
        slides.setMode(true);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Intake flapper = new Intake(hardwareMap);
        Pose2d startPose = new Pose2d(33, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence firstDrive = drive.trajectorySequenceBuilder(startPose)
                .forward(3)
                .strafeLeft(13)
                .lineToLinearHeading(new Pose2d(11, -34.2, Math.toRadians(121.25)))
                .forward(5)
                .build();
        Trajectory toJunction = drive.trajectoryBuilder(firstDrive.end())
                .back(.01)
                .build();
        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toJunction.end())
                .back(16)
                .lineToLinearHeading(new Pose2d(13.5,-33.2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(13.5, -26.7, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(70, -4, Math.toRadians(0)), Math.toRadians(-1.75))
                .addTemporalMarker(1.5, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(854);

                    // Run your action in here!
                })
                .addTemporalMarker(4.5, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .build();
        TrajectorySequence scoreStack = drive.trajectorySequenceBuilder(toStack.end())
                .back(18)
                .addTemporalMarker(.2,() -> {
                    slides.move(7350);
                })
                .lineToLinearHeading(new Pose2d(36.5, -8.6, Math.toRadians(121.25)))
                .build();
        TrajectorySequence toJunctionHigh = drive.trajectorySequenceBuilder(scoreStack.end())
                .forward(3)
                .build();
        Trajectory toStackAfter = drive.trajectoryBuilder(scoreStack.end())
                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(0)))
                .build();
        TrajectorySequence zone3Strafe = drive.trajectorySequenceBuilder(toJunction.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(57, -31, Math.toRadians(90)))
                .addTemporalMarker(2, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })

                .build();

        TrajectorySequence zone1Strafe = drive.trajectorySequenceBuilder(toJunction.end())
                .back(4.5)
                .lineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)))
                .addTemporalMarker(2, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })
                .build();

        TrajectorySequence zone2Strafe = drive.trajectorySequenceBuilder(toJunction.end())
                .back(4.5)
                .lineToLinearHeading(new Pose2d(33, -34, Math.toRadians(90)))
                .addTemporalMarker(2, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })
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
                FtcDashboard.getInstance().startCameraStream(camera, 5);

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
        slides.control(-448, 3, 1);
        flapper.move(Intake.state.CLOSE);
        sleep(1000);
        slides.control(1200, 3, 1);
        drive.followTrajectorySequenceAsync(firstDrive);
        slides.move(7350);
        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PRELOAD:
                    telemetry.addLine("PRELOAD");
                    if (!drive.isBusy() && !slides.isBusy()) {
                        state = DRIVE_STATE.FIRST_TRAJ;
                        drive.followTrajectoryAsync(toJunction);

                    }
                    break;
                case FIRST_TRAJ:
                    telemetry.addLine("FIRST_TRAJ");

                    if(!drive.isBusy() && !slides.isBusy()) {
                        sleep(750);
                        flapper.move(Intake.state.OPEN);
                        sleep(1000);
                        flapper.move(Intake.state.CLOSE);
                        state = DRIVE_STATE.STACK_FIRST;
                    }
                    break;
                case STACK_FIRST:
                    telemetry.addLine("STACK FIRST");
                    if (!drive.isBusy() && !slides.isBusy()){
                        drive.followTrajectorySequenceAsync(toStack);
                        state = DRIVE_STATE.SCORE_STACK;
                    }
                    break;
                case SCORE_STACK:
                    telemetry.addLine("SCORE STACk");
                    if(!drive.isBusy() && !slides.isBusy()){
                        flapper.move(Intake.state.CLOSE);
                        slides.control(1000,5,1);
                        drive.followTrajectorySequenceAsync(scoreStack);
                        state = DRIVE_STATE.toJunction;
                    }
                    break;
                case SCORE_STACK2:
                    if(!drive.isBusy() && !slides.isBusy()){
                        flapper.move(Intake.state.OPEN);
                        sleep(1000);
                        pauseTime = runtime.time();
                        state = DRIVE_STATE.SCORE_STACK3;
                    }
                    break;
                case toJunction:
                    if(!drive.isBusy() && !slides.isBusy()){
                        drive.followTrajectorySequenceAsync(toJunctionHigh);
                        state = DRIVE_STATE.SCORE_STACK2;
                    }
                    break;
                case SCORE_STACK3:
                    if(runtime.time() - pauseTime > 2){
                        state = DRIVE_STATE.PARK;
                    }
                    break;
                case SEC_TRAJ:
                    telemetry.addLine("SEC_TRAJ");
                    if(!slides.isBusy() && !drive.isBusy()) {

                        if (parking_zone == 1) {
                            drive.followTrajectorySequenceAsync(zone1Strafe);
                        } else if (parking_zone == 3) {
                            drive.followTrajectorySequenceAsync(zone3Strafe);
                        } else {
                            drive.followTrajectorySequenceAsync(zone2Strafe);
                        }
                        state = DRIVE_STATE.PARK;
                    }
                    break;
                case PARK:
                    telemetry.addLine("PARK");

                    if (!drive.isBusy() && !slides.isBusy()) {
                        state = DRIVE_STATE.IDLE;
                        flapper.move(Intake.state.CLOSE);
                    }
                    break;
                case IDLE:
                    telemetry.addLine("IDLE");

                    break;
            }

            drive.update();
            slides.update();
            telemetry.addData("pos", slides.getCurrentPos());
            telemetry.update();
        }
    }
}


