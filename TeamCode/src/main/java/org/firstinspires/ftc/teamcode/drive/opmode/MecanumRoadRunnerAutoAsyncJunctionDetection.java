package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.JunctionPipeline;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@Autonomous(name = "Mecanum Roadrunner Async To Junction", preselectTeleOp = "Mecanum Power Play Teleop")
public class MecanumRoadRunnerAutoAsyncJunctionDetection extends LinearOpMode {
    public static int parking_zone = 2;
    public static double zone3X = 20;
    final double flapDown = .77;
    public static double zone1Y = 31;
    public static double zone1X = 17;
    public double pauseTime = 0;
    public int conesLeft = 5;
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
        STACK_SECOND,
        CYCLE,
        STACK,
        IDLE
    }

    DRIVE_STATE state = DRIVE_STATE.PRELOAD;
    OpenCvCamera aprilTagCam;
    OpenCvCamera junctionCam;
    AprilTagPipeline aprilTagDetectionPipeline;
    JunctionPipeline junctionDetectionPipeline;
    final public MecanumBotConstant names = new MecanumBotConstant();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    HashMap<Integer, Integer> tagDict = new HashMap<Integer, Integer>();
    final int ID_TAG_OF_INTEREST = 0;
    final int ID_TAG_OF_INTEREST_2 = 4;
    final int ID_TAG_OF_INTEREST_3 = 7;
    public static double P = 72;
    public static double I = 16;
    public static double D = 8;
    public static double turns = 0;
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
                .build();
        Trajectory toJunction = drive.trajectoryBuilder(firstDrive.end().plus(new Pose2d(0,0,Math.toRadians(turns))))
                .forward(6)
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toJunction.end())
                .waitSeconds(.75)
                .back(8)
                .lineToLinearHeading(new Pose2d(14,-33.2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14, -26.7, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(68, -7.75, Math.toRadians(0)), Math.toRadians(-2.5))
                .addDisplacementMarker(.25,0 , () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(454);
                    // Run your action in here!
                })
                .addDisplacementMarker(.5,0, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .build();
        TrajectorySequence scoreStack = drive.trajectorySequenceBuilder(toStack.end())
                .lineToConstantHeading(new Vector2d(50,-8))
                .lineToLinearHeading(new Pose2d(36.5, -8.6, Math.toRadians(117.25)))
                .build();
        TrajectorySequence toJunctionHigh = drive.trajectorySequenceBuilder(scoreStack.end())
                .forward(5.625)
                .build();
        TrajectorySequence scoreStack2 = drive.trajectorySequenceBuilder(toJunctionHigh.end())
                .back(5.625)
                .lineToLinearHeading(new Pose2d(68, -7.75, Math.toRadians(0)))
                .addDisplacementMarker(.05,0, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .addDisplacementMarker(.15,0 , () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(310);
                    // Run your action in here!
                })
                .addDisplacementMarker(.5,0, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .build();

        TrajectorySequence zone3Strafe = drive.trajectorySequenceBuilder(toJunctionHigh.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(6.5)
                .lineToLinearHeading(new Pose2d(57, -8, Math.toRadians(90)))
                .addDisplacementMarker(.9,0, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })
                .build();

        TrajectorySequence zone1Strafe = drive.trajectorySequenceBuilder(toJunctionHigh.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(6.5)
                .lineToLinearHeading(new Pose2d(23, -8, Math.toRadians(90)))
                .addDisplacementMarker(.9,0, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })
                .build();

        TrajectorySequence zone2Strafe = drive.trajectorySequenceBuilder(toJunctionHigh.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(6.5)
                .lineToLinearHeading(new Pose2d(42, -8, Math.toRadians(90)))
                .addDisplacementMarker(.9,0, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(0);

                    // Run your action in here!
                })
                .build();

        tagDict.put(0, 1);
        tagDict.put(7, 2);
        tagDict.put(4, 3);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        aprilTagCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, names.cameraApril), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(TAGSIZE, fx, fy, cx, cy);

        aprilTagCam.setPipeline(aprilTagDetectionPipeline);
        aprilTagCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                aprilTagCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        junctionCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, names.cameraJunction));
        junctionDetectionPipeline = new JunctionPipeline(telemetry);
        junctionCam.setPipeline(junctionDetectionPipeline);
        junctionCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                junctionCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        flapper.move(Intake.state.CLOSE);
        sleep(1000);
        slides.init();
        telemetry.addLine("Slides are good");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

//Initializing Loop
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
//        sleep(5250);

//Init seqeuence
        state = DRIVE_STATE.PRELOAD;
//        slides.control(1200, 3, 1);
        drive.followTrajectorySequenceAsync(firstDrive);
        slides.move(4300);

        //Main Loop
        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PRELOAD:
                    telemetry.addLine("PRELOAD");
                    if (!drive.isBusy() && !slides.isBusy()) {
                        if(junctionDetectionPipeline.getLocation() == null){
                            state = DRIVE_STATE.FIRST_TRAJ;
                            telemetry.addLine("Target");
                            telemetry.update();
                            drive.followTrajectoryAsync(toJunction);
                            break;
                        }
                        switch (junctionDetectionPipeline.getLocation()){
                            case TARGET:
                                state = DRIVE_STATE.FIRST_TRAJ;
                                telemetry.addLine("Target");
                                telemetry.update();
                                toJunction = drive.trajectoryBuilder(firstDrive.end().plus(new Pose2d(0,0,Math.toRadians(turns))))
                                        .forward(6)
                                        .build();
                                drive.followTrajectoryAsync(toJunction);
                                break;
                            case UP:
                                telemetry.addLine("UP");
                                telemetry.update();
                                drive.turn(Math.toRadians(-7.5));
                                turns -= 7.5;
                                break;
                            case BELOW:
                                telemetry.addLine("Below");
                                telemetry.update();
                                drive.turn(Math.toRadians(7.5));
                                turns += 7.5;
                                break;
                        }



                    }
                    break;
                case FIRST_TRAJ:
                    telemetry.addLine("FIRST_SCORE");

                    if(!drive.isBusy() && !slides.isBusy()) {
                        slides.control(4000, 5,1);
                        sleep(750);
                        flapper.move(Intake.state.OPEN);
                        slides.control(4300, 5,1);
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

                        slides.move(4400);
                        state = DRIVE_STATE.toJunction;
                    }
                    break;
                case toJunction:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(toJunctionHigh);
                        state = DRIVE_STATE.SCORE_STACK2;

                    }
                    break;
                case SCORE_STACK2:
                    if(!drive.isBusy() && !slides.isBusy()){
                        slides.control(3800, 5, 1);
                        flapper.move(Intake.state.OPEN);
                        sleep(1000);
                        slides.control(4400, 5, 1);
                        pauseTime = runtime.time();
                        state = DRIVE_STATE.SCORE_STACK3;
                    }
                    break;
                case SCORE_STACK3:
                    if(!drive.isBusy() && !slides.isBusy()){
                        conesLeft--;
                        if(conesLeft == 4){
                            state = DRIVE_STATE.SEC_TRAJ;
                        }
                        else {
                            drive.followTrajectorySequenceAsync(scoreStack2);
                            state = DRIVE_STATE.toJunction;
                        }
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
//    public void checkCCW(){
//        switch (junctionDetectionPipeline.getLocation()){
//            case UP:
//
//        }


//    }

}


