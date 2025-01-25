package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeeptesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, -64, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(7, -32), Math.toRadians(270))
                        .setReversed(true)
                        .strafeTo(new Vector2d(7, -35))
                        .splineToLinearHeading(new Pose2d(41, -11, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(41, -54))
                        .strafeTo(new Vector2d(50, -11))
                        .strafeTo(new Vector2d(50, -54))
                        .strafeTo(new Vector2d(50, -50))
                        .strafeTo(new Vector2d(32, -50))
                        .strafeTo(new Vector2d(32, -54))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}