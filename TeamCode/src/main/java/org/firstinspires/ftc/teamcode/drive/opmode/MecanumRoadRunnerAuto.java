package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.MecanumBot;
import org.firstinspires.ftc.teamcode.PowerplayBot;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@Autonomous(name = "Mecanum Roadrunner")
public class MecanumRoadRunnerAuto extends LinearOpMode {
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


    // UNITS ARE METERS
    final double TAGSIZE = 0.05;
    private ElapsedTime runtime = new ElapsedTime();


    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() throws InterruptedException{
        Lift slides = new Lift(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Intake flapper = new Intake(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence firstDrive = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(15)

                .lineToLinearHeading(new Pose2d(-21.5,28,Math.toRadians(121.25)))
                .build();

        Trajectory toJunction = drive.trajectoryBuilder(firstDrive.end())
                .forward(5)
                .build();


        Trajectory zone3Strafe = drive.trajectoryBuilder(toJunction.end())
                .lineToLinearHeading(new Pose2d(zone3X, zone1Y, Math.toRadians(90)))
                .build();

        Trajectory zone1Strafe = drive.trajectoryBuilder(toJunction.end())
                .lineToLinearHeading(new Pose2d(-zone1X, zone1Y, Math.toRadians(90)))
                .build();

        Trajectory zone2Strafe = drive.trajectoryBuilder(toJunction.end())
                .lineToLinearHeading(new Pose2d(0, zone1Y, Math.toRadians(90)))
                .build();



        tagDict.put(0,1);
        tagDict.put(7,2);
        tagDict.put(4,3);
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
        slides.control(500, 3, 1);
        flapper.move(flapUp);
        sleep(750);
        slides.control(-600, 2, 1);
        flapper.move(flapDown);
        sleep(500);
        slides.control(500, 3, 1);
        drive.followTrajectorySequence(firstDrive);
        slides.control(6933,8,1);
        drive.followTrajectory(toJunction);

        flapper.move(flapUp);

        sleep(1000);

        if (parking_zone == 1) {
            drive.followTrajectory(zone1Strafe);
        } else if (parking_zone == 3) {
            drive.followTrajectory(zone3Strafe);
        } else {
            drive.followTrajectory(zone2Strafe);
        }
        slides.reset();

    }







    }


class Intake{
    MecanumBot m = new MecanumBot();
    DcMotorSimple flapper;
    public Intake(HardwareMap hardwareMap){
        flapper = hardwareMap.get(DcMotorSimple.class, m.intake);
    }
    public void move(double position){
        flapper.setPower(position);
    }
}
class Lift {
    MecanumBot m = new MecanumBot();
    DcMotor slides;
    NormalizedColorSensor color;
    double targetPos;
    public Lift(HardwareMap hardwareMap) {
        slides = hardwareMap.get(DcMotor.class, m.slides);
        color = hardwareMap.get(NormalizedColorSensor.class, m.color);


    }
    public boolean isBusy(){
        return slides.getPower() != 0;
    }
    public void init(){
        color.setGain((float) 15);
        slides.setPower(.7);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < .42 && colors.red < .42){

            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control(-650, 2,.7);
    }
    public void reset(){
        color.setGain((float) 15);
        slides.setPower(-.7);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < .42 && colors.red < .42){

            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void control(double target, double timeoutS, double SLIDE_POWER) {
        double currentTime = System.currentTimeMillis();
        double slidesPosition = slides.getCurrentPosition();
        if (slidesPosition > target) {
            slides.setPower(-SLIDE_POWER);
            while (slidesPosition > target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        } else if (slidesPosition < target) {
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
    }
    public void move(double target){
        targetPos = target;
    }
    public void update() {
        control(targetPos, 6,1);
    }
}