package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
                        .strafeTo(new Vector2d(-38, -26)) // drive to preload shooting position
                        .waitSeconds(1) // shoot preloads
                        //.setTangent(0)
                                .splineTo(new Vector2d(12, -50), Math.toRadians(270))
                                .splineTo(new Vector2d(12, -50), Math.toRadians(270))
                        //.splineToLinearHeading(new Pose2d(6, -25, Math.toRadians(260)), Math.toRadians(0)) // get ready to intake at second spike mark
                        //.splineToLinearHeading(new Pose2d(12, -50, Math.toRadians(270)), Math.toRadians(270)) // drive to intake position of second spike mark
                        .setReversed(true)
                        .splineTo(new Vector2d(-16, -12), Math.toRadians(90)) // drive to second spike mark shooting position
                        .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot second spike mark
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineTo(new Vector2d(24, -14), Math.toRadians(25)) // prepare to open gate and intake
                        .setReversed(false)
                        .splineTo(new Vector2d(10, -58), Math.toRadians(240)) // open gate and intake
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(12, -26), Math.toRadians(90)) // prepare to shoot gate
                        .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot gate
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(255)) // prepare to intake first spike mark
                        .strafeToLinearHeading(new Vector2d(-12, -50), Math.toRadians(270)) // intake first spike mark
                        .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot first spike mark
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineTo(new Vector2d(50, -20), Math.toRadians(0)) // prepare to intake third spike mark
                        .setReversed(false)
                        .splineTo(new Vector2d(36, -50), Math.toRadians(270)) // intake third spike mark
                        .setReversed(true)
                        .splineTo(new Vector2d(-15, -15), Math.toRadians(135)) // prepare to shoot third spike mark
                        .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot third spike mark
                        .waitSeconds(1)
                .build());

        /* 15 ARTIFACT AUTO TESTING
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -46, Math.toRadians(235)))
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // drive to shooting position for preloads
                .waitSeconds(1) // shoot preloads
                .setReversed(false)
                .splineTo(new Vector2d(-13, -51), Math.toRadians(270)) // intake at first spike mark
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // go to close shoot position
                .waitSeconds(1) // shoot second time (pre-placed 1)
                .setReversed(false)
                .splineTo(new Vector2d(-4, -53), Math.toRadians(270)) // open gate
                .waitSeconds(1) // keep gate open
                .setReversed(true)
                .splineTo(new Vector2d(0, -16), Math.toRadians(90)) // prepare to intake at second spike mark
                .setReversed(false)
                .splineTo(new Vector2d(11.5, -51), Math.toRadians(270)) // intake at second spike mark
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)) // go to close shoot position
                .waitSeconds(1) // shoot third time (pre-placed 2)
                .turnTo(Math.toRadians(210)) // turn so there is no interference with the opposite side
                .setReversed(true)
                .splineTo(new Vector2d(34.5, -15), Math.toRadians(45)) // prepare to intake at third spike mark
                .setReversed(false)
                .splineTo(new Vector2d(34.5, -51), Math.toRadians(270)) // intake at third spike mark
                .setReversed(true)
                .splineTo(new Vector2d(54, -14), Math.toRadians(25)) // go to far shoot position
                .waitSeconds(1) // shoot fourth time (pre-placed 3)
                .setReversed(false)
                .splineTo(new Vector2d(58, -60), Math.toRadians(270)) // intake at hp zone
                .setReversed(true)
                .splineTo(new Vector2d(54, -14), Math.toRadians(25)) // go to far shoot position
                .waitSeconds(1) // shoot fifth time (hp zone)
                .build()); */

        /* ORIGINAL 9+3 ARTIFACT AUTO */
        /* myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, -34, Math.toRadians(232.2)))
                .strafeTo(new Vector2d(-46, -22))
        .strafeToLinearHeading(new Vector2d(-14, -5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-14, -28), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(232.2))
                .strafeToLinearHeading(new Vector2d(10, -5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(10, -28), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(232.2))
                .strafeToLinearHeading(new Vector2d(34, -5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(34, -28), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(232.2))
                .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(0))

                .build()); */

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}