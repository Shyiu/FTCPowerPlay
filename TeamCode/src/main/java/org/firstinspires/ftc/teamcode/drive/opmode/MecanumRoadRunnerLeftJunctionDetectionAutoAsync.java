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
import org.firstinspires.ftc.teamcode.JunctionPipeline;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@Autonomous(name = "Final Mecanum Left Auto Async", preselectTeleOp = "Mecanum Power Play Teleop")
public class MecanumRoadRunnerLeftJunctionDetectionAutoAsync extends LinearOpMode {
    public static int parking_zone = 2;
    final Intake.state flapUp = (Intake.state.OPEN);
    final Intake.state flapDown = (Intake.state.CLOSE);
    public static int[] STACK_LEVEL = {504, 304, 104, -204, -500};
    public static double dist1x = -12.5;
    public static double dist2x = -17;
    public static double dist1y = -29.78;
    public static double dist2y = -5.35;
    public double pauseTime = 0;
    public static double turns = 0;
    public int conesLeft = 5;
    public int cones = 3;
    int exitCount = 0;

    enum DRIVE_STATE {
        PRELOAD,
        FIRST_TRAJ,
        SEC_TRAJ,
        PARK,
        STACK_FIRST,
        SCORE_STACK,
        toJunction,
        toJunction2,
        CYCLE_OR_EXIT,
        SCORE_STACK2,
        STACK_SECOND,
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
        Pose2d startPose = new Pose2d(-33, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence firstDrive = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-19, -61.72, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-10.75, -31.78, Math.toRadians(90)), Math.toRadians(90.0),
                        drive.getVelocityConstraint(DriveConstants.MAX_VEL * .90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0.025, 0, () -> {
                    slides.move(4300);
                })
                .build();
        Trajectory toJunction = drive.trajectoryBuilder(firstDrive.end())
                .forward(4.5)
                .build();
        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toJunction.end())
                .back(3)
                .lineToLinearHeading(new Pose2d(dist1x, dist1y, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(dist2x, dist2y, Math.toRadians(90)), Math.toRadians(90.0),
                        drive.getVelocityConstraint(DriveConstants.MAX_VEL * .90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .forward(55, drive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(.25, 0, () -> {
                    slides.move(STACK_LEVEL[0]);
                })
                .addDisplacementMarker(.5, 0, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .build();
        TrajectorySequence moveToJunction = drive.trajectorySequenceBuilder(toStack.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .waitSeconds(.125)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    slides.control(1100, 1, 1);
                })
                .back(54)
                .addDisplacementMarker(.03, 8, () -> {
                    slides.move(4400);
                })
                .turn(Math.toRadians(-50))
                .build();
//        Trajectory toStackAfter = drive.trajectoryBuilder(scoreStack.end())
        TrajectorySequence scoreStack = drive.trajectorySequenceBuilder(moveToJunction.end())
                .forward(8.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                    slides.control(4000, 5, 1);
                    flapper.move(Intake.state.OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    slides.control(4300, 5, 1);
                    flapper.move(Intake.state.CLOSE);
                })
                .build();
        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(scoreStack.end())
                .back(8.5)
                .turn(Math.toRadians(50))
                .forward(52,
                        drive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(.125, 0, () -> {
                    slides.move(STACK_LEVEL[1]);
                })
                .addDisplacementMarker(.5, 0, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .build();

        TrajectorySequence moveToJunction2Zone1 = drive.trajectorySequenceBuilder(toStack2.end())
                .back(54)
                .addDisplacementMarker(.03, 8, () -> {
                    slides.move(4400);
                })
                .turn(Math.toRadians(-50))
                .build();
        TrajectorySequence moveToJunction2Zone2 = drive.trajectorySequenceBuilder(toStack2.end())
                .back(30)
                .addDisplacementMarker(.03, 8, () -> {
                    slides.move(4450);
                })
                .turn(Math.toRadians(-135))
                .build();
        TrajectorySequence scoreStack2Zone2 = drive.trajectorySequenceBuilder(moveToJunction2Zone2.end())
                .forward(8.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                    slides.control(4000, 5, 1);
                    flapper.move(Intake.state.OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(.28, () -> {
                    slides.control(4350, 5, 1);
                    flapper.move(Intake.state.CLOSE);
                })
                .build();
        TrajectorySequence moveToJunction2Zone3 = drive.trajectorySequenceBuilder(toStack2.end())
                .back(8)
                .addDisplacementMarker(0, 8, () -> {
                    slides.move(1942);
                })
                .turn(Math.toRadians(135))
                .build();
        TrajectorySequence scoreStack2Zone3 = drive.trajectorySequenceBuilder(moveToJunction2Zone3.end())
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                    slides.control(1700, 5, 1);
                    flapper.move(Intake.state.OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(.28, () -> {
                    slides.control(1900, 5, 1);
                    flapper.move(Intake.state.CLOSE);
                })
                .build();
        TrajectorySequence scoreStack2Zone1 = drive.trajectorySequenceBuilder(moveToJunction2Zone1.end())
                .forward(8.5)
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                    slides.control(4100, 5, 1);
                    flapper.move(Intake.state.OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(.28, () -> {
                    slides.control(4350, 5, 1);
                    flapper.move(Intake.state.CLOSE);
                })
                .build();
        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(scoreStack2Zone1.end())
                .back(8.5)
                .turn(Math.toRadians(50), 5.5, 4.2)
                .addDisplacementMarker(.125, 0, () -> {
                    slides.move(STACK_LEVEL[2]);
                })
                .addDisplacementMarker(.5, 0, () -> {
                    flapper.move(Intake.state.OPEN);
                })
                .forward(54)
                .build();

        TrajectorySequence[] seqs = {toStack2, toStack3};
        int seqIndex = 0;

        TrajectorySequence zone3Strafe = drive.trajectorySequenceBuilder(scoreStack2Zone3.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(8)
                .turn(Math.toRadians(-35), 5, 5)
//                .strafeLeft(12)
                .addDisplacementMarker(.4, 0, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(STACK_LEVEL[3]);
                    // Run your action in here!
                })
                .build();

        TrajectorySequence zone1Strafe = drive.trajectorySequenceBuilder(scoreStack2Zone1.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(8)
                .turn(Math.toRadians(-40), 5, 5)
                .addDisplacementMarker(.4, 0, () -> {
                    // This marker runs two seconds into the trajectory

                    slides.move(STACK_LEVEL[3]);
                    // Run your action in here!
                })
                .build();

        TrajectorySequence zone2Strafe = drive.trajectorySequenceBuilder(scoreStack2Zone2.end())
                .addTemporalMarker(.2, () -> {
                    flapper.move(Intake.state.CLOSE);
                })
                .back(8.5)
                .turn(Math.toRadians(38), 5, 5)
                .addDisplacementMarker(.4, 0, () -> {
                    // This marker runs two seconds into the trajectory
                    slides.move(STACK_LEVEL[3]);
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
                FtcDashboard.getInstance().startCameraStream(aprilTagCam, 5);

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
        sleep(125);
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

        //Main Loop
        while (opModeIsActive() && !isStopRequested()) {
            switch (state) {
                case PRELOAD:
                    telemetry.addLine("PRELOAD");
                    if (!drive.isBusy() && !slides.isBusy()) {
                        if (junctionDetectionPipeline.getLocation() == null || exitCount == 3) {
                            state = DRIVE_STATE.FIRST_TRAJ;
                            telemetry.addLine("Target");
                            telemetry.update();
                            exitCount = 0;
                            toJunction = drive.trajectoryBuilder(firstDrive.end().plus(new Pose2d(0, 0, Math.toRadians(turns))))
                                    .forward(4.5)
                                    .build();
                            drive.followTrajectoryAsync(toJunction);
                            break;
                        }
                        switch (junctionDetectionPipeline.getLocation()) {
                            case TARGET:
                                state = DRIVE_STATE.FIRST_TRAJ;
                                telemetry.addLine("Target");
                                telemetry.update();
                                toJunction = drive.trajectoryBuilder(firstDrive.end().plus(new Pose2d(0, 0, Math.toRadians(turns))))
                                        .forward(4.5)
                                        .build();
                                drive.followTrajectoryAsync(toJunction);
                                break;
                            case UP:
                                telemetry.addLine("UP");
                                telemetry.update();
                                drive.turn(Math.toRadians(-2.5));
                                turns -= 2.5;
                                exitCount++;
                                break;
                            case BELOW:
                                telemetry.addLine("Below");
                                telemetry.update();
                                drive.turn(Math.toRadians(2.5));
                                turns += 2.5;
                                exitCount++;
                                break;
                        }
                    }
                    break;
                case FIRST_TRAJ:
                    telemetry.addLine("FIRST_SCORE");

                    if (!drive.isBusy() && !slides.isBusy()) {
                        slides.control(4000, 5, 1);
                        flapper.move(Intake.state.OPEN);
                        slides.control(4300, 5, 1);
                        sleep(125);
                        flapper.move(Intake.state.CLOSE);

                        state = DRIVE_STATE.STACK_FIRST;
                    }
                    break;
                case STACK_FIRST:
                    telemetry.addLine("STACK FIRST");
                    if (!drive.isBusy() && !slides.isBusy()) {
                        drive.followTrajectorySequenceAsync(toStack);
                        state = DRIVE_STATE.SCORE_STACK;
                    }
                    break;
                case SCORE_STACK:
                    telemetry.addLine("SCORE STACk");
                    if (!drive.isBusy() && !slides.isBusy()) {
                        drive.followTrajectorySequenceAsync(moveToJunction);
                        state = DRIVE_STATE.toJunction;
                    }
                    break;
                case toJunction:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(scoreStack);
                        state = DRIVE_STATE.CYCLE_OR_EXIT;
                    }
                    break;
                case CYCLE_OR_EXIT:
                    if (!drive.isBusy() && !slides.isBusy()) {
                        conesLeft--;
                        if (conesLeft == cones) {
                            state = DRIVE_STATE.SEC_TRAJ;
                        } else {
                            drive.followTrajectorySequenceAsync(seqs[seqIndex]);
                            seqIndex += 1;
                            state = DRIVE_STATE.SCORE_STACK2;
                        }
                    }
                    break;

                case SCORE_STACK2:
                    if (!drive.isBusy() && !slides.isBusy()) {
                        sleep(62);
                        flapper.move(Intake.state.CLOSE);
                        sleep(100);
                        slides.control(1100, 1, 3);
                        if (parking_zone == 1) {
                            drive.followTrajectorySequenceAsync(moveToJunction2Zone1);
                        } else if (parking_zone == 2) {
                            drive.followTrajectorySequenceAsync(moveToJunction2Zone2);
                        } else if (parking_zone == 3) {
                            drive.followTrajectorySequenceAsync(moveToJunction2Zone3);
                        }
                        state = DRIVE_STATE.toJunction2;
                    }
                    break;
                case toJunction2:
                    if (!drive.isBusy() && !slides.isBusy()) {
                        if (parking_zone == 1) {
                            drive.followTrajectorySequenceAsync(scoreStack2Zone1);
                        } else if (parking_zone == 2) {
                            drive.followTrajectorySequenceAsync(scoreStack2Zone2);
                        } else if (parking_zone == 3) {
                            drive.followTrajectorySequenceAsync(scoreStack2Zone3);
                        }
                        state = DRIVE_STATE.CYCLE_OR_EXIT;
                    }
                    break;

                case SEC_TRAJ:
                    telemetry.addLine("SEC_TRAJ");


                    if (!slides.isBusy() && !drive.isBusy()) {

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

