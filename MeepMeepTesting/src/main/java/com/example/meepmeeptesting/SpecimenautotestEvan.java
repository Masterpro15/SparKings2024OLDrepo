package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenautotestEvan {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(90)))

                //.afterTime(0.2, arm.armNewSpec())
               // .afterTime(0.1, claw.clawClose())
               // .afterTime(0.1, wrist.wristNewScore())
               // .afterTime(0.1, lift.liftScoreSpec())
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(0, -29.45))
                .waitSeconds(0.05)
               // .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.05)
              //  .afterTime(0 , wrist.wristDown())
             //   .afterTime(0, lift.liftDown())

                .setReversed(true)
                //.afterTime(0 , arm.armStop())
                .splineToSplineHeading(new Pose2d(new Vector2d(27,-43), Math.toRadians(90)), 0)
              //  .afterTime(0 , arm.armDown())
              //  .afterTime(0.1 , arm.armStop())
                .splineToLinearHeading(new Pose2d(new Vector2d(43,-13), Math.toRadians(90)), 0)
                .strafeTo(new Vector2d(47.5, -13))


                //.splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(47.5,-53))
                //one in observation zone
                //.strafeTo(new Vector2d(45,-13))

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(new Vector2d(58, -13), Math.toRadians(-90)), 0)

                //  .strafeTo(new Vector2d(56.5,-13))
               // .afterTime(0, wrist.wristGrab())
                .strafeTo(new Vector2d(56.8,-58))
                .waitSeconds(0.5)
              //  .afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
               // .afterTime(0.1 , arm.armNewSpec())
              //  .afterTime(0.1, wrist.wristNewScore())
               // .afterTime(0.1, lift.liftScoreSpec())
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                .waitSeconds(0.05)
               // .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.1)
               // .afterTime(0 , wrist.wristGrab())
              //  .afterTime(0, lift.liftDown())
                .setReversed(true)
               // .afterTime(0 , arm.armDown())
                //.afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(43, -57), Math.toRadians(270)), Math.toRadians(270))
               /// .afterTime(0, wrist.wristGrab())
               // .afterTime(0 , arm.armStop())

               // .afterTime(0, wrist.wristGrab())

                .waitSeconds(0.5)
                //.afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
              //  .afterTime(0.1 , arm.armNewSpec())
               // .afterTime(0.1, wrist.wristNewScore())
               // .afterTime(0.1, lift.liftScoreSpec())



                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                .waitSeconds(0.05)
                // .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.1)
                // .afterTime(0 , wrist.wristGrab())
                //  .afterTime(0, lift.liftDown())
                .setReversed(true)
                // .afterTime(0 , arm.armDown())
                //.afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(43, -56), Math.toRadians(270)), Math.toRadians(270))
                /// .afterTime(0, wrist.wristGrab())
                // .afterTime(0 , arm.armStop())

                // .afterTime(0, wrist.wristGrab())

                .waitSeconds(0.5)
                //.afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
                //  .afterTime(0.1 , arm.armNewSpec())
                // .afterTime(0.1, wrist.wristNewScore())
                // .afterTime(0.1, lift.liftScoreSpec())






                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                .waitSeconds(0.05)
               // .afterTime(0.1 , claw.clawOpen())
               // .afterTime(0 , wrist.wristDown())
              //  .afterTime(0, lift.liftDown())
                .waitSeconds(0.05)

                        .setReversed(true)
                .splineToLinearHeading(new Pose2d( new Vector2d(60, -61), Math.toRadians(-270)), 0)



                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}