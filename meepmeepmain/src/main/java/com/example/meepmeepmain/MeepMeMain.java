package com.example.meepmeepmain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeMain {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 60, 1.97, Math.toRadians(60), 15.31)
                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(new Pose2d(-37.5, -62, Math.toRadians(90)))
//                                        .strafeRight(26)
//                                        .lineToLinearHeading(new Pose2d(-11.5, -34.2, Math.toRadians(121.25-90)))
//                                        .splineToSplineHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(200))
//                                        .lineToLinearHeading(new Pose2d(-33.1, -8.6, Math.toRadians(38)))
//                                        .splineToSplineHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(200))
//                                        .lineToLinearHeading(new Pose2d(-33.1, -8.6, Math.toRadians(38)))
//                                        .splineToSplineHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(200))
//                                        .lineToLinearHeading(new Pose2d(-33.1, -8.6, Math.toRadians(38)))
//                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                        drive.trajectorySequenceBuilder(new Pose2d(33, -62, Math.toRadians(90)))
                                .strafeLeft(15)
                                .lineToLinearHeading(new Pose2d(11.5, -34.2, Math.toRadians(121.25)))
                                .splineToSplineHeading(new Pose2d(57, -12, Math.toRadians(0)), Math.toRadians(-15))
                                .lineToLinearHeading(new Pose2d(33.1, -8.6, Math.toRadians(128)))
                                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(33.1, -8.6, Math.toRadians(128)))
                                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(33.1, -8.6, Math.toRadians(128)))
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}