package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_COLLECT =  100 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final int  LIFT_SCORING_IN_HIGH_BASKET = (int) (600 * LIFT_TICKS_PER_MM);
    final double TINY = ( 400 * LIFT_TICKS_PER_MM);

    double liftPosition = LIFT_COLLAPSED;
    private DcMotorEx liftMotor;
    private OpMode myOpMode;
    public Lift(OpMode opmode) {
        myOpMode = opmode;
    }
    int armposition;
    public void init(){
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void init2(){
        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) liftMotor).setVelocity(2100);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public class liftbackvel implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            ((DcMotorEx) liftMotor).setVelocity(2100);

            return false;
        }
    }
    public Action liftVel() {
        return new liftbackvel();
    }
    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(0.8);
                initialized = true;
            }

            double pos = liftMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 1945) {
                return true;
            } else {
                liftMotor.setPower(0);
                return false;
            }
        }
    }
    public Action liftUp() {
        return new LiftUp();
    }






    public class LiftTiny implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(0.8);
                initialized = true;
            }

            double pos = liftMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 1340) {
                return true;
            } else {
                liftMotor.setPower(0);
                return false;
            }
        }
    }
    public Action liftTiny() {
        return new LiftTiny();
    }




    public class LiftScoreSpec implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(0.8);
                initialized = true;
            }

            double pos = liftMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 762) {
                return true;
            } else {
                liftMotor.setPower(0);
                return false;
            }
        }
    }
    public Action liftScoreSpec() {
        return new LiftScoreSpec();
    }






    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(-0.8);
                initialized = true;
            }

            double pos = liftMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 10) {
                return true;
            } else {
                liftMotor.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }

}
