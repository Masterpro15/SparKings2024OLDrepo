package org.firstinspires.ftc.teamcode.auto;


// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Wrist;

@Config
@Autonomous(name = "SpecimenAUTONEW", group = "Autonomous")
public class newSpecAuto extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Claw claw= new Claw(this);
    Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(0, -61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder build = drive.actionBuilder(startPose)

                .afterTime(0, arm.armNewSpec2())
                 .afterTime(0, claw.clawClose())
                 .afterTime(0, wrist.wristNewScore())
                 .afterTime(0.1, lift.liftScoreSpec())
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(0, -29))
                .waitSeconds(0.05)
                 .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.05)
                 .afterTime(0 , wrist.wristDown())
                .afterTime(0, lift.liftDown())

                .setReversed(true)
                .afterTime(0 , arm.armStop())
                .splineToSplineHeading(new Pose2d(new Vector2d(27,-43), Math.toRadians(-90)), 0)
                 .afterTime(0 , arm.armDown())
                 .afterTime(0.1 , arm.armStop())
                .splineToLinearHeading(new Pose2d(new Vector2d(43,-13), Math.toRadians(-90)), 0)
                .strafeTo(new Vector2d(47.5, -13))



                .strafeTo(new Vector2d(47.5,-53))
                //one in observation zone
                //.strafeTo(new Vector2d(45,-13))

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(new Vector2d(58, -13), Math.toRadians(-90)), 0)

                 .afterTime(0, wrist.wristGrab())
                .strafeTo(new Vector2d(56.8,-58))
                .waitSeconds(0.5)
                  .afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
                 .afterTime(0.1 , arm.armNewSpec())
                  .afterTime(0.1, wrist.wristNewScore())
                 .afterTime(0.1, lift.liftScoreSpec())
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                .waitSeconds(0.05)
                 .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.1)
                 .afterTime(0 , wrist.wristGrab())
                 .afterTime(0, lift.liftDown())
                .setReversed(true)
                .afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(43, -54.5), Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0 , arm.armDown())
                 .afterTime(0, wrist.wristGrab())
                 .afterTime(0 , arm.armStop())

                 .afterTime(0, wrist.wristGrab())

                .waitSeconds(0.5)
                .afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
                  .afterTime(0.1 , arm.armNewSpec())
                 .afterTime(0.1, wrist.wristNewScore())
                 .afterTime(0.1, lift.liftScoreSpec())



                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                .waitSeconds(0.05)
                 .afterTime(0.1 , claw.clawOpen())
                .waitSeconds(0.1)
                 .afterTime(0 , wrist.wristGrab())
                  .afterTime(0, lift.liftDown())
                .setReversed(true)
                .afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(43, -54.5), Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0 , arm.armDown())
                .afterTime(0, wrist.wristGrab())
                .afterTime(0 , arm.armStop())

                .afterTime(0, wrist.wristGrab())

                .waitSeconds(0.5)
                .afterTime(0.1 , claw.clawClose())
                .waitSeconds(0.1)
                  .afterTime(0.1 , arm.armNewSpec())
                 .afterTime(0.1, wrist.wristNewScore())
                 .afterTime(0.1, lift.liftScoreSpec())






                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(1,-30), Math.toRadians(-270)), Math.toRadians(90))
                 .afterTime(0.1 , claw.clawOpen())
                 .afterTime(0 , wrist.wristDown())
                .afterTime(0, lift.liftDown())
                .setReversed(true)
                .afterTime(0 , arm.armDown())

                .splineToLinearHeading(new Pose2d( new Vector2d(60, -61), Math.toRadians(-270)), 0)










                ;
        arm.init();
        wrist.init();
        claw.init();
        lift.init2();
        int position= 1;
        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(

            ));


        }

        waitForStart();
        Actions.runBlocking(new ParallelAction(
                build.build()
        ));









    }

}

