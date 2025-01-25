package org.firstinspires.ftc.teamcode.auto;

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
public class highBasketAuto extends LinearOpMode {
    private Pose2d startPose = null;
    private MecanumDrive drive = null;
    private Arm arm = null;
    private Wrist wrist = null;
    private Lift lift = null;
    private Claw claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        startPose = new Pose2d(0, -61, Math.toRadians(90)); // Starting pose
        drive = new MecanumDrive(hardwareMap, startPose);   // Initialize Mecanum Drive
        arm = new Arm(this);                                // Initialize Arm
        wrist = new Wrist(this);                            // Initialize Wrist
        lift = new Lift(this);                              // Initialize Lift
        claw = new Claw(this);                              // Initialize Claw

        // Initialize subsystems
        arm.init();
        wrist.init();
        lift.init();
        claw.init();

        // Build trajectory actions
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
                .afterTime(0, arm.armBasket())// Arm up to score basket
                .afterTime(0, lift.liftHighBasket())
                .afterTime(0, wrist.wristScore())//wrist up so it doesn't hit anything
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))//go to position to score (-54.5,-54.5)
                .afterTime(0, wrist.wristMid())// claw can go in the basket
                .afterTime(0, claw.clawOpen())//claw opens and we get first high basket:)
                .strafeTo(new Vector2d(-50, -50))
                .strafeTo(new Vector2d(-48, -61))// Move to (-48, -61) to grab
                .afterTime(0, arm.armCollapse())   //collapse from the high basket score
                .afterTime(0, wrist.wristDown())  // Move wrist down to get ready to collect
                .afterTime(0, claw.clawOpen())    // Open the claw
                .afterTime(0, arm.armGrab())     // Move arm to grab position so we can get close
                .afterTime(0, lift.liftTiny())   // Lift up slightly to pick sample up
                .afterTime(0, claw.clawClose())  // Close the claw
                .waitSeconds(1.5)                // Wait for it to complete in case some thing happens stop here
                .afterTime(0, arm.armBasket())
                .afterTime(0, lift.liftHighBasket())
                .strafeTo(new Vector2d(-48, -48)) // Move to basket position location
                .strafeTo(new Vector2d(-50, -50))
                .afterTime(0, wrist.wristScore()) // Move wrist to scoring position
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))//go to the basket
                .afterTime(0, wrist.wristMid())   // Move wrist to mid position
                .afterTime(0, claw.clawOpen())    // Open the claw Yay second one in :)
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0, arm.armCollapse())  // Collapse the arm
                .afterTime(0, wrist.wristDown())  // Move wrist down
                .afterTime(0, claw.clawOpen())    // Open claw again
                .afterTime(0, arm.armGrab())      // Move arm to grab position
                .afterTime(0, lift.liftTiny())    // Lift up slightly
                .afterTime(0, claw.clawClose())   // Close the claw again
                .waitSeconds(1.5)                 // Wait for actions to complete
                .afterTime(0, arm.armBasket())// Move arm to basket position
                .afterTime(0, lift.liftHighBasket())
                .afterTime(0, wrist.wristScore()) // Move wrist to scoring position
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))
                .afterTime(0, wrist.wristMid())   // Move wrist to mid position
                .afterTime(0, claw.clawOpen())    // Open claw
                .strafeTo(new Vector2d(-50, -50))
                .splineToLinearHeading(new Pose2d(-52, 0, Math.toRadians(-180)), Math.toRadians(-180))
                .afterTime(0, arm.armCollapse())  // Collapse the arm
                .setReversed(true)                // Reverse movement direction
                .strafeTo(new Vector2d(-25, 0)); // Strafe to final position

        // Wait for the start signal
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            // Run the built trajectory
            Actions.runBlocking(new SequentialAction(builder.build()));

            // Additional telemetry to show that we got high basket Yay!
            telemetry.addData("Status", "Road Runner for High Basket Complete");
            telemetry.update();
        }
    }
}
