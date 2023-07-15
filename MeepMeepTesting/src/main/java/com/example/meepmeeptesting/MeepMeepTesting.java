package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15.75, 14)
                .setConstraints(52, 52, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.02, -64.43, Math.toRadians(90.25)))
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .setReversed(true)
                                .splineTo(new Vector2d(-60.35, -11.66), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .setReversed(true)
                                .splineTo(new Vector2d(-60.35, -11.66), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .setReversed(true)
                                .splineTo(new Vector2d(-60.35, -11.66), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .setReversed(true)
                                .splineTo(new Vector2d(-60.35, -11.66), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .setReversed(true)
                                .splineTo(new Vector2d(-60.35, -11.66), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-28.61, -7.64), Math.toRadians(55.54))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}