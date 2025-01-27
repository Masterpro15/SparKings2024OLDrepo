package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    private OpMode myOpMode;

    public Wrist(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0); // Set wrist to the down position
            return false;
        }
    }

    public Action wristDown() {
        return new WristDown();
    }

    public class WristScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.67); // Set wrist to the scoring position
            return false;
        }
    }

    public Action wristScore() {
        return new WristScore();
    }

    public class WristMid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.5); // Set wrist to the mid position
            return false;
        }
    }

    public Action wristMid() {
        return new WristMid();
    }
}
