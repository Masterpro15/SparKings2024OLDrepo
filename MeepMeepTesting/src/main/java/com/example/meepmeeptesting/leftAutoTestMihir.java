package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class leftAutoTestMihir {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, -61, Math.toRadians(90)))

                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225)) // Move to (-54.5, -54.5) at 225°
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-48, -61, Math.toRadians(90)), Math.toRadians(90)) // Move to (-54.5, -54.5) at 225°

                .strafeTo(new Vector2d(-48, -61)) // Move to (-48, -61)
                .strafeTo(new Vector2d(-48, -48)) // Move to (-48, -48)
                .waitSeconds(1.5) // Wait for 1.5 seconds
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225)) // Move to (-54.5, -54.5) at 225°
                .waitSeconds(1.5) // Wait for 1.5 seconds
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(90)) // Move to (-50, -50)
                .waitSeconds(1.5) // Wait for 1.5 seconds
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-52, 0, Math.toRadians(-180)), Math.toRadians(-180))
                .setReversed(true)
                .strafeTo(new Vector2d(-25, 0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
