package com.example.meepmeepmain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeMain {
    public static double dist1x = 12.5;
    public static double dist2x = 17;
    public static double dist1y = -29.78;
    public static double dist2y = -6.05;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 60, 1.97, Math.toRadians(60), 15.31)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33.01, -62.00, Math.toRadians(90.00)))
                                .lineToLinearHeading(new Pose2d(19, -61.72, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(10.75, -31.28, Math.toRadians(90)), Math.toRadians(90))
                                .forward(5)
                                .back(3)
                                .lineToLinearHeading(new Pose2d(dist1x, dist1y, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(dist2x, dist2y, Math.toRadians(90)), Math.toRadians(90.0))
                                .splineToLinearHeading(new Pose2d(57.86,0, Math.toRadians(0)), Math.toRadians(0))
                                .build()

                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}