package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    // Constants
    private static final double ARM_TICKS_PER_DEGREE =
            28 // Number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // Exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // External gear reduction (20T driving 100T hub gear)
                    * 1 / 360.0; // Convert to ticks per degree

    // Target positions
    private static final int ARM_COLLAPSED_INTO_ROBOT = (int) (0 * ARM_TICKS_PER_DEGREE);
    private static final int ARM_SCORE_SPECIMEN = (int) (60 * ARM_TICKS_PER_DEGREE);
    private static final int ARM_SCORE_SPECIMEN2 = (int) (25 * ARM_TICKS_PER_DEGREE);
    private static final int ARM_COLLECT_SAMPLE = (int) (20 * ARM_TICKS_PER_DEGREE);
    private static final int ARM_HIGH_BASKET = (int) (90 * ARM_TICKS_PER_DEGREE);

    private DcMotorEx armMotor;
    private final OpMode myOpMode;

    public Arm(OpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        armMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Action: Move arm down
    public class ArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(ARM_COLLAPSED_INTO_ROBOT);
            return false;
        }
    }

    public Action armDown() {
        return new ArmDown();
    }

    // Action: Move arm up
    public class ArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(ARM_SCORE_SPECIMEN);
            return false;
        }
    }

    public Action armUp() {
        return new ArmUp();
    }

    // Action: Grab position
    public class ArmGrab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(ARM_COLLECT_SAMPLE);
            return false;
        }
    }

    public Action armGrab() {
        return new ArmGrab();
    }

    // Action: Stop the arm
    public class ArmStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setPower(0);
            return false;
        }
    }

    public Action armStop() {
        return new ArmStop();
    }

    public class ArmBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(ARM_HIGH_BASKET);
            return false;
        }
    }

    public Action armBasket() {
        return new ArmBasket();
    }

    public class ArmCollapse implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            moveToPosition(ARM_COLLAPSED_INTO_ROBOT);
            return false;
        }
    }

    public Action armCollapse() {
        return new ArmCollapse();
    }

    // Helper method to set the arm position
    private void moveToPosition(int position) {
        armMotor.setTargetPosition(position);
        armMotor.setPower(0.7); // Adjust speed as necessary
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
