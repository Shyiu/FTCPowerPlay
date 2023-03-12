package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.obselete.PowerplayBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * TODO: Calibrate the PID so the slides stop as close to the target as possible. The target is outputed to telemetry. Use Dashboard to edit the values quickly.
 * TODO: Being cycling off of the cone stack. In order to do use lineToHeading or something like that when making the spline path so the robot drives straight and turns to the correct heading. Move the slides at the same time.
 * TODO: Slides.control moves the slides while pausing the code
 * TODO: Slides.move will move the slides async as long as slides.update is in a loop
 * */

@Config
@Autonomous(name = "Roadrunner Trajectory Testing")
public class MecanumDriveTrajTesting extends LinearOpMode {



    final public PowerplayBot names = new PowerplayBot();
    public static double dist1x = 12.5;
    public static double dist2x = 17;
public static double dist1y = -29.78;
public static double dist2y = -5.55;
public static double dist3y = -6.05;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(33, -62, Math.toRadians(90));
        TrajectorySequence firstDriveBetter = drive.trajectorySequenceBuilder(new Pose2d(33.0, -62.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(19, -61.72, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(10.75, -31.28, Math.toRadians(90)), Math.toRadians(90.0),
                        drive.getVelocityConstraint(DriveConstants.MAX_VEL*.90 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5)
                .back(3)
                .lineToLinearHeading(new Pose2d(dist1x, dist1y, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(dist2x, dist2y, Math.toRadians(90)), Math.toRadians(90.0),
                        drive.getVelocityConstraint(DriveConstants.MAX_VEL*.90 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(-90))
                .forward(54,drive.getVelocityConstraint(DriveConstants.MAX_VEL*1.4 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToLinearHeading(new Pose2d(20, -35.06, Math.toRadians(90.00)), Math.toRadians(2.32))
//                .lineTo(new Vector2d(20.01, -14.23))
//                .splineTo(new Vector2d(28.70, -11.46), Math.toRadians(-10.75))
//                .splineTo(new Vector2d(63.25, -12.01), Math.toRadians(2.48))

                .build();

        drive.setPoseEstimate(startPose);

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

//        This loop is initializing the camera and trying to identify an AprilTag using AprilTagPipeline

        waitForStart();
        drive.followTrajectorySequence(firstDriveBetter);
        telemetry.update();

    }
}


