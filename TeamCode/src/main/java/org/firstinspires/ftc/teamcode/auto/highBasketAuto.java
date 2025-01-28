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
        startPose = new Pose2d(-25, -61, Math.toRadians(90)); // Starting pose
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

                // Move to the high basket position (-54.5, -54.5)
                .afterTime(0, claw.clawClose()) // Close the claw to secure the object
                .afterTime(0, arm.armBasket())  // Move the arm to basket position
                .afterTime(0, lift.liftTiny())  // Slightly lift the object
                .afterTime(0, wrist.wristScore()) // Move wrist to scoring position
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))

                // Score the object in the high basket
                .afterTime(0, wrist.wristMid()) // Adjust wrist to mid position for scoring
                .afterTime(0, claw.clawOpen()) // Open the claw to release the object

                // Reset to prepare for the next object
                .afterTime(0, wrist.wristScore()) // Move wrist back to safe position
                .strafeTo(new Vector2d(-50, -50))
                .splineToLinearHeading(new Pose2d(-48, -61, Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0, lift.liftDown())
                .afterTime(0, arm.armCollapse()) // Collapse the arm
                .afterTime(0, wrist.wristDown()) // Move wrist down to get ready to collect

                // Simulate grabbing the next object
                .afterTime(0, claw.clawOpen())
                .afterTime(0, arm.armGrab())
                .afterTime(0, lift.liftTiny())
                .afterTime(0, claw.clawClose())
                .waitSeconds(1.5)

                // Move back to high basket for the second score
                .afterTime(0, arm.armBasket())
                .afterTime(0, lift.liftUp())
                .afterTime(0, wrist.wristScore())
                .strafeTo(new Vector2d(-48, -61))
                .strafeTo(new Vector2d(-48, -48))

                // Score the second object in the high basket
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))
                .afterTime(0, wrist.wristMid())
                .afterTime(0, claw.clawOpen())

                // Move to the next position
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0, arm.armCollapse())
                .afterTime(0, wrist.wristDown())

                // Simulate grabbing another object
                .afterTime(0, claw.clawOpen())
                .afterTime(0, arm.armGrab())
                .afterTime(0, lift.liftTiny())
                .afterTime(0, claw.clawClose())
                .waitSeconds(1.5)

                // Move back to high basket position
                .afterTime(0, arm.armBasket())
                .afterTime(0, lift.liftHighBasket())
                .afterTime(0, wrist.wristScore())
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225))
                .afterTime(0, wrist.wristMid())
                .afterTime(0, claw.clawOpen())

                // Final movements
                .strafeTo(new Vector2d(-50, -50))
                .splineToLinearHeading(new Pose2d(-43, 0, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, lift.liftDown())
                .afterTime(0, arm.armCollapse())
                .strafeTo(new Vector2d(-25, 0));

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
