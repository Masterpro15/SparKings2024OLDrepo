package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(225)))
                //.splineToLinearHeading(new Pose2d(new Vector2d(-49.5, -46), Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(new Vector2d(40,-13), Math.toRadians(-90)), 0)
                .strafeTo(new Vector2d(47.5, -13))

                .strafeTo(new Vector2d(47.5,-53))
                //one in observation zone
                //.strafeTo(new Vector2d(45,-13))

                .setReversed(true)


                .splineToLinearHeading(new Pose2d(new Vector2d(58, -13), Math.toRadians(-90)), 0)


                //put arm up while strafing
                //stop and place the sample on the bar
                //.afterTime(0.2, claw.autonOpenClaw())
                //.afterTime(0.3, viper.autonSlightOut())


/*
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)
                //.afterTime(0, shoulder.autonDown())

                .splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))
                .strafeTo(new Vector2d(55,-13))
                //.strafeTo(new Vector2d(43,-59))
                //undo ^ if something goes wrong.
                .strafeTo(new Vector2d(46,-60))

                //.afterTime(0, claw.autonCloseClaw())
                .waitSeconds(0.3)
                //.afterTime(0, shoulder.autonHC())
                //grab sample, routing towards chamber.
                //raise arm to clip
                //.afterTime(1.4, viper.autonHangSpecimen())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(7, -30), Math.toRadians(90)), Math.toRadians(90))


                //clip, routing to push final sample and grab specimen

                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0.1, viper.autonSlightOut())



                //.afterTime(1, shoulder.autonDown())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(35, -59.5), Math.toRadians(-80)), Math.toRadians(0))

                //.afterTime(0, claw.autonCloseClaw())
                .waitSeconds(0.3)
                //.afterTime(0, shoulder.autonHC())
                //grab sample, routing towards chamber.
                //raise arm to clip
                //.afterTime(1.5, viper.autonHangSpecimen())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(4, -30), Math.toRadians(90)), Math.toRadians(90))

                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0, viper.autonSlightOut())


                //.afterTime(1, shoulder.autonDown())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(35, -59.5), Math.toRadians(-80)), Math.toRadians(0))

                //.afterTime(0, claw.autonCloseClaw())
                .waitSeconds(0.3)
                //.afterTime(0, shoulder.autonHC())
                //grab sample, routing towards chamber.
                //raise arm to clip
                //.afterTime(1.5, viper.autonHangSpecimen())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(6, -30), Math.toRadians(90)), Math.toRadians(90))

                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0, viper.autonSlightOut())
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(50,-60), Math.toRadians(-90))
                //.afterTime(0, shoulder.autonDown())
                 /*.strafeTo(new Vector2d(0, -30))
                .waitSeconds(1)

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)


                .splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))

                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(55,-59))
                .waitSeconds(0.5)

                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(270)), Math.toRadians(270))
                        .waitSeconds(0.5)
                        .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(55,-57), Math.toRadians(270)), Math.toRadians(270))


                */
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}