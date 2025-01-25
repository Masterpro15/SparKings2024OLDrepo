package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_COLLECT = 100 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 600 * LIFT_TICKS_PER_MM;
    final double TINY = 600 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;
    private DcMotorEx liftMotor;
    private OpMode myOpMode;

    public Lift(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class LiftTiny implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftMotor.setTargetPosition((int) TINY);
            liftMotor.setPower(0.6); // Adjust speed as necessary
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action liftTiny() {
        return new LiftTiny();
    }

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftMotor.setTargetPosition((int) LIFT_COLLAPSED);
            liftMotor.setPower(0.6); // Adjust speed as necessary
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public class LiftHighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftMotor.setTargetPosition((int) LIFT_SCORING_IN_HIGH_BASKET);
            liftMotor.setPower(0.6); // Adjust speed as necessary
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Check if target is reached
            int currentPosition = liftMotor.getCurrentPosition();
            if (currentPosition >= LIFT_SCORING_IN_HIGH_BASKET - 10 && currentPosition <= LIFT_SCORING_IN_HIGH_BASKET + 10) { // Within tolerance
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return true; // Action complete
            }
            return false; // Action ongoing
        }
    }

    public Action liftHighBasket() {
        return new LiftHighBasket();
    }
}