package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    // Constants for lift movement (Ticks per mm calculation)
    private static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    // Target positions
    private static final int LIFT_COLLAPSED = (int) (0 * LIFT_TICKS_PER_MM);
    private static final int LIFT_COLLECT = (int) (100 * LIFT_TICKS_PER_MM);
    private static final int LIFT_SCORING_IN_LOW_BASKET = (int) (300 * LIFT_TICKS_PER_MM);
    private static final int LIFT_SCORING_IN_HIGH_BASKET = (int) (2100 * LIFT_TICKS_PER_MM);
    private static final int LIFT_TINY = (int) (2100 * LIFT_TICKS_PER_MM);

    private DcMotorEx liftMotor;
    private final OpMode myOpMode;

    public Lift(OpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Action: Move lift to high basket
    public class LiftHighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_SCORING_IN_HIGH_BASKET);
            return false;
        }
    }

    public Action liftHighBasket() {
        return new LiftHighBasket();
    }

    // Action: Move lift to low basket
    public class LiftLowBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_SCORING_IN_LOW_BASKET);
            return false;
        }
    }

    public Action liftLowBasket() {
        return new LiftLowBasket();
    }

    // Action: Move lift up
    public class LiftUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_SCORING_IN_HIGH_BASKET);
            return false;
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }

    // Action: Move lift down (collapsed)
    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_COLLAPSED);
            return false;
        }
    }

    public Action liftDown() {
        return new LiftDown();
    }

    // Action: Move lift to a small adjustment (Tiny position)
    public class LiftTiny implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_TINY);
            return false;
        }
    }

    public Action liftTiny() {
        return new LiftTiny();
    }

    // Action: Stop the lift
    public class LiftStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftMotor.setPower(0);
            return false;
        }
    }

    public Action liftStop() {
        return new LiftStop();
    }

    // Helper method to move lift to target position
    private void moveToPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setPower(0.8); // Adjust power as needed
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Debugging telemetry
        myOpMode.telemetry.addData("Lift Moving To:", targetPosition);
        myOpMode.telemetry.addData("Current Lift Position:", liftMotor.getCurrentPosition());
        myOpMode.telemetry.update();
    }
}
