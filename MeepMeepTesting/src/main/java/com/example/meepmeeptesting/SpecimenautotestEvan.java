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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(90)))

                /*.afterTime(0.2, arm.armUp())
                .afterTime(0.1, claw.clawClose())

                 */
                .strafeTo(new Vector2d(0, -31.4))
                //.afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))
                /*.afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())

                 */
                .waitSeconds(0.8)
                /*.afterTime(0 , wrist.wristDanger())
                /* .afterTime(0, arm.armSpec2())
                 .afterTime(0, wrist.wristDanger())
                 .afterTime(0, wrist.wristScore())
                 .afterTime(0 , wrist.wristScore2())
                 .afterTime(0, wrist.wristDanger())
                 .waitSeconds(2)

                 */
               // .afterTime(1, claw.clawOpen())

                //first specimen done
                .waitSeconds(0.8)
                //.afterTime(0.1, wrist.wristDown())

                .setReversed(true)
                //.afterTime(0 , arm.armStop())
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)
                .splineToLinearHeading(new Pose2d(new Vector2d(40,-13), Math.toRadians(-90)), 0)
                                .strafeTo(new Vector2d(46, -13))


                //.splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(46,-53))
                //one in observation zone
                //.strafeTo(new Vector2d(45,-13))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(56.5, -13), Math.toRadians(-90)), 0)

                //  .strafeTo(new Vector2d(56.5,-13))
               // .afterTime(0, wrist.wristGrab())
                .strafeTo(new Vector2d(56.5,-54.5))
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(-2,-31.25), Math.toRadians(-270)), Math.toRadians(90))
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(43, -52.345), Math.toRadians(270)), Math.toRadians(270))
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(2, -31.25), Math.toRadians(-270)), Math.toRadians(90))
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(43, -52.345), Math.toRadians(270)), Math.toRadians(270))



                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}