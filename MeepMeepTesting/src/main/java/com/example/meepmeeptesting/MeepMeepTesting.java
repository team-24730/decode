package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.9145755) // default: 60, 60, 180, 180, 15
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -46, Math.toRadians(235)))
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // drive to shooting position for preloads
                .waitSeconds(1) // shoot preloads
                .setReversed(false)
                .splineTo(new Vector2d(-13, -51), Math.toRadians(270)) // intake pre-placed 1
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // go to shoot position
                .waitSeconds(1) // shoot second time (pre-placed 1)
                .setReversed(false)
                .splineTo(new Vector2d(-4, -53), Math.toRadians(270)) // open gate
                .waitSeconds(1) // keep gate open
                .setReversed(true)
                .splineTo(new Vector2d(0, -16), Math.toRadians(90)) // back position for intake pre-placed 2
                .setReversed(false)
                .splineTo(new Vector2d(11.5, -51), Math.toRadians(270)) // intake pre-placed 2
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // go to shoot position
                .waitSeconds(1) // shoot third time (pre-placed 2)
                .turnTo(Math.toRadians(210))
                .setReversed(true)
                .splineTo(new Vector2d(34.5, -15), Math.toRadians(45)) // back position for intake pre-placed 3
                .setReversed(false)
                .splineTo(new Vector2d(34.5, -51), Math.toRadians(270)) // intake pre-placed 3
                .setReversed(true)
                .splineTo(new Vector2d(54, -14), Math.toRadians(25)) // go to shoot position
                .waitSeconds(1) // shoot fourth time (pre-placed 3)
                .setReversed(false)
                .splineTo(new Vector2d(58, -60), Math.toRadians(270)) // intake hp zone
                .setReversed(true)
                .splineTo(new Vector2d(54, -14), Math.toRadians(25)) // go to shoot position
                .waitSeconds(1) // shoot fifth time (hp zone)
                .build());


        /* myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -12.3, Math.toRadians(-180)))
                .splineTo(new Vector2d(55, -13.5), Math.toRadians(210)) // drive to first shooting position
                .waitSeconds(1) // shoot preloads
                .turnTo(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50, -58), Math.toRadians(290)) // drive over to pick up preloads
                .strafeToLinearHeading(new Vector2d(60, -58), Math.toRadians(0)) // intake the preloads
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(50, -10), Math.toRadians(-180))
                .setReversed(false)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-140))
                .build()); */

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}