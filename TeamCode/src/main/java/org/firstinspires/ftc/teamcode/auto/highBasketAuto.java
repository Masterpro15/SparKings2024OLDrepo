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
public class highBasketAuto extends LinearOpMode {

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

                // Step 1: Move to the high basket position
                .afterTime(0.1, claw.clawClose())                     // Close the claw to secure object
                .afterTime(0.1, arm.armBasket())                      // Move the arm to basket position
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225)) // Move to basket
                .afterTime(0, lift.liftUp())                // Lift to high basket height
                .afterTime(0, wrist.wristScore())                   // Prepare wrist for scoring


                // Step 2: Score the object in the high basket
                .afterTime(0, wrist.wristMid())                     // Adjust wrist for scoring
                .afterTime(0, claw.clawOpen())                     // Open the claw to release object

                //: Reset for the next object
                .strafeTo(new Vector2d(-50, -50))                   // Back off from the basket
                .splineToLinearHeading(new Pose2d(-48, -61, Math.toRadians(90)), Math.toRadians(90)) // Move to reset area
                .afterTime(0, lift.liftDown())                      // Lower the lift
                .afterTime(0, arm.armCollapse())                    // Collapse the arm
                .afterTime(0, wrist.wristDown())                    // Move wrist to reset position

                // === Step 4: Simulate grabbing the next object ===
                .afterTime(0, claw.clawOpen())                      // Open the claw
                .afterTime(0, arm.armGrab())                        // Move the arm to grab position
                .afterTime(0, lift.liftTiny())                      // Slight lift adjustment
                .afterTime(0, claw.clawClose())                     // Close claw to grab object
                .waitSeconds(1.5)                                   // Wait to secure object

                // Score the second object
                .afterTime(0, arm.armBasket())                      // Move arm to basket position
                .afterTime(0, lift.liftUp())                // Lift to high basket height
                .afterTime(0, wrist.wristScore())                   // Prepare wrist for scoring
                .splineToLinearHeading(new Pose2d(-54.5, -54.5, Math.toRadians(225)), Math.toRadians(225)) // Move to basket
                .afterTime(0, wrist.wristMid())                     // Adjust wrist for scoring
                .afterTime(0, claw.clawOpen())                     // Open claw to release object

                // Step 6: Final Park Position
                .strafeTo(new Vector2d(-50, -50))                   // Move away from basket
                .splineToLinearHeading(new Pose2d(-43, 0, Math.toRadians(0)), Math.toRadians(0)) // Move to parking area
                .afterTime(0, lift.liftDown())                      // Lower the lift
                .afterTime(0, arm.armCollapse())                    // Collapse the arm
                .strafeTo(new Vector2d(-25, 0));                    // Final parking position

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
