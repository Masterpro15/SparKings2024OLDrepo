package org.firstinspires.ftc.teamcode.auto;

// Import necessary libraries
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Wrist;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name = "HighBasketAuto", group = "Autonomous")
public class HighBasjet extends LinearOpMode {

    // Declare hardware and starting pose
    private Pose2d startPose = new Pose2d(-25, -61, Math.toRadians(90));
    private MecanumDrive drive = null;
    private Arm arm = null;
    private Wrist wrist = null;
    private Lift lift = null;
    private Claw claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // === Initialize Hardware ===
        drive = new MecanumDrive(hardwareMap, startPose);   // Mecanum Drive
        arm = new Arm(this);                                // Arm
        wrist = new Wrist(this);                            // Wrist
        lift = new Lift(this);                              // Lift
        claw = new Claw(this);                              // Claw

        // Initialize all subsystems
        arm.init();
        wrist.init();
        lift.init2();
        claw.init();

        // Trajectory Building
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)

                .afterTime(0.1, claw.clawClose())                     // Close the claw to secure object
                .afterTime(0.1, arm.armBasket())
                .splineToLinearHeading(new Pose2d(-59.5, -59.5, Math.toRadians(225)), Math.toRadians(225))
                .afterTime(0, lift.liftUp())
                .afterTime(0, wrist.wristMid())                   // Prepare wrist for scoring
                .waitSeconds(0.9)
                .afterTime(0.1, wrist.wristScore())
                .waitSeconds(0.3)
                .afterTime(0.3, claw.clawOpen())
                .waitSeconds(0.8)
                .afterTime(0, wrist.wristDown())
                .afterTime(0.2, lift.liftDown())
                .waitSeconds(1)
                .afterTime(0.1, arm.armDown())
                .afterTime(0.1 , arm.armStop())
                .splineToLinearHeading(new Pose2d(new Vector2d(-51.5, -46), Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0.1 , arm.armStop())
                .afterTime(0 , wrist.wristGrabSample())
                .waitSeconds(1)
                .afterTime(0, lift.liftTiny())
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(0.5)
                .afterTime(0 , lift.liftDown())
                .afterTime(0 , wrist.wristDown())
                .waitSeconds(0.5)
                .afterTime(0.1, arm.armBasket())
                .afterTime(0, lift.liftUp())
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(-59.5, -59.5, Math.toRadians(225)), Math.toRadians(225))

                .waitSeconds(0.5)
                .afterTime(0.3, wrist.wristScore())
                .waitSeconds(1)
                .afterTime(0.3, claw.clawOpen())
                .waitSeconds(0.4)
                .afterTime(0, wrist.wristDown())
                .afterTime(0.2, lift.liftDown())
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-53, -53))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d( -62.67, -46))

                .afterTime(0, arm.armDown())
                .waitSeconds(0.5)





                .afterTime(0.1 , arm.armStop())
                .afterTime(0 , wrist.wristGrabSample())
                .waitSeconds(2)
                .afterTime(0, lift.liftTiny())
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(1)
                .afterTime(0 , lift.liftDown())
                .afterTime(0 , wrist.wristDown())
                .waitSeconds(0.5)
                .afterTime(0.1, arm.armBasket())
                .afterTime(0, lift.liftUp())
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(-59.5, -59.5, Math.toRadians(225)), Math.toRadians(225))

                .afterTime(0, wrist.wristMid())                   // Prepare wrist for scoring
                .waitSeconds(0.4)
                .afterTime(0, wrist.wristScore())                     // Adjust wrist for scoring
                .waitSeconds(1)
                .afterTime(0, claw.clawOpen())
                .waitSeconds(0.3)
                .afterTime(0, wrist.wristDown())
                .afterTime(0.2, lift.liftDown())
                .splineToLinearHeading(new Pose2d(new Vector2d(-59.5f, -46), Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0, arm.armDown())
                .strafeTo(new Vector2d(-53, -30))
                .waitSeconds(3)




                ;
                // Open the claw to release object

                         // Final parking position

        //  Wait for Start Signal =
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }

        waitForStart();

        // === Execute the Autonomous Routine ===
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(builder.build()));

            // Log telemetry to indicate routine completion
            telemetry.addData("Status", "High Basket Auto Routine Complete!");
            telemetry.update();
        }
    }
}
