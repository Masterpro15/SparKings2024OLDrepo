package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo wrist;
    private OpMode myOpMode;
    public Claw(OpMode opmode) {
        myOpMode = opmode;
    }
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        wrist = myOpMode.hardwareMap.get(Servo.class, "claw");

    }
    public class ClawClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(1);
            return false;
        }
    }

    public Action clawClose() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            wrist.setPosition(0);
            return false;
        }
    }

    public Action clawOpen() {
        return new ClawOpen();
    }


}