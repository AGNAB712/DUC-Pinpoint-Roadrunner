package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(32, 64, Math.toRadians(270)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(45))
                .strafeTo(new Vector2d(61, 61))
                .strafeToLinearHeading(new Vector2d(46.5, 45), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(45))
                .strafeTo(new Vector2d(61, 61))
                .strafeToLinearHeading(new Vector2d(56, 54), Math.toRadians(224))
                .strafeToLinearHeading(new Vector2d(56, 45), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(45))
                .strafeTo(new Vector2d(61, 61))
                .strafeToLinearHeading(new Vector2d(53, 28), Math.toRadians(1))
                .strafeTo(new Vector2d(60.5, 28))
                .strafeToLinearHeading(new Vector2d(45, 20), Math.toRadians(180))
                .strafeTo(new Vector2d(35, 20))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}