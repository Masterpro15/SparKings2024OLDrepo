package org.firstinspires.ftc.teamcode.auto;


// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "3TESTAutoSpecimen", group = "Autonomous")
public class threeSpecAuto extends LinearOpMode {
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
                .afterTime(0.2, arm.armUp())
                .afterTime(0.1, claw.clawClose())
                .strafeTo(new Vector2d(0, -31.434))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0.1 , arm.armSpec3())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.5)
                .afterTime(0.1 , wrist.wristDanger())
                /* .afterTime(0, arm.armSpec2())
                 .afterTime(0, wrist.wristDanger())
                 .afterTime(0, wrist.wristScore())
                 .afterTime(0 , wrist.wristScore2())
                 .afterTime(0, wrist.wristDanger())
                 .waitSeconds(2)

                 */
                .waitSeconds(0.1)
                .afterTime(1, claw.clawOpen())

                //first specimen done
                .waitSeconds(0.3)
                .afterTime(0.1, wrist.wristDown())
                .setReversed(true)
                .afterTime(0 , arm.armStop())
                /*.strafeTo(new Vector2d(0, -31.4))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))

                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0 , wrist.wristDanger())
                .afterTime(0 , arm.armSpec3())
                */




               /* .afterTime(0, arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0, wrist.wristScore())
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(2)
                */


                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)
                .afterTime(0 , lift.liftVel())

                .splineToLinearHeading(new Pose2d(new Vector2d(45,-13), Math.toRadians(-90)), 0)

                .afterTime(0, wrist.wristGrab())

                .strafeTo(new Vector2d(45,-55))
                //one in observation zone
                // testing .strafeTo(new Vector2d(45,-13))
                // next spline to go faster
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(0.9)
                .afterTime(1, arm.armUp())
                .afterTime(0, wrist.wristDown())
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(-2,-31.25), Math.toRadians(-270)), Math.toRadians(90))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0, claw.clawOpen())
                .afterTime(0, wrist.wristDown())
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .afterTime(0 , arm.armStop())
                .afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(43, -52.345), Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0, wrist.wristGrab())
                //2 done
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(0.9)
                .afterTime(1, arm.armUp())
                .afterTime(0, wrist.wristDown())

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(1.6,-26), Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -26))//31-30
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0, claw.clawOpen())
                .afterTime(0, wrist.wristDown())
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .afterTime(0 , arm.armStop())
                .afterTime(0, wrist.wristGrab())
                .strafeTo(new Vector2d(50, -54.6))
                .afterTime(0, wrist.wristGrab())

                /*
                .


                .splineToSplineHeading(new Pose2d(new Vector2d(55,-55), Math.toRadians(270)), Math.toRadians(270))


            */








                ;
        arm.init();
        wrist.init();
        claw.init();
        lift.init();
        int position= 1;
        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(

            ));


        }

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                build.build()
        ));









    }

}

