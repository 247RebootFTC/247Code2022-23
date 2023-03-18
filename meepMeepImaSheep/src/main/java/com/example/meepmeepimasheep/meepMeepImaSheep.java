package com.example.meepmeepimasheep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeepImaSheep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -60, Math.toRadians(-90)))
                                .back(40)
                                .UNSTABLE_addDisplacementMarkerOffset(40, () -> {

                                    //CAMERA DETECTION

                                })
                                .splineToSplineHeading(new Pose2d(35.5, -13, Math.toRadians(240)), Math.toRadians(90))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(32, -21, Math.toRadians(196)), Math.toRadians(60))
                                .UNSTABLE_addTemporalMarkerOffset(3, () -> {

                                    //1+5 (mid) HOLY MOLY

                                })
                                .waitSeconds(2.5)
                                .setReversed(false)
                                .splineTo(new Vector2d(35.5,-12.5), Math.toRadians(-90))
                                //.strafeLeft(24) ZONE 3
                                //.strafeRight(24) ZONE 1
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}