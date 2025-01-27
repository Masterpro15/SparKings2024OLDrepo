package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    private static final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_COLLECT = 100 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORING_IN_HIGH_BASKET = 600 * LIFT_TICKS_PER_MM;
    private static final double TINY = 600 * LIFT_TICKS_PER_MM;

    private DcMotorEx liftMotor;
    private final OpMode myOpMode;

    public Lift(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveToPosition(double targetPosition) {
        liftMotor.setTargetPosition((int) targetPosition);
        liftMotor.setPower(0.6); // Adjust speed as necessary
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean isAtTarget(double targetPosition) {
        int currentPosition = liftMotor.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= 10;
    }

    public class LiftTiny implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(TINY);
            return isAtTarget(TINY);
        }
    }

    public Action liftTiny() {
        return new LiftTiny();
    }

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_COLLAPSED);
            return isAtTarget(LIFT_COLLAPSED);
        }
    }

    public Action liftDown() {
        return new LiftDown();
    }

    public class LiftHighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_SCORING_IN_HIGH_BASKET);
            if (isAtTarget(LIFT_SCORING_IN_HIGH_BASKET)) {
                liftMotor.setPower(0.1); // Hold position gently
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return true; // Action complete
            }
            return false; // Action ongoing
        }
    }

    public Action liftHighBasket() {
        return new LiftHighBasket();
    }

    public class LiftUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(LIFT_SCORING_IN_HIGH_BASKET);
            return isAtTarget(LIFT_SCORING_IN_HIGH_BASKET);
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }
}