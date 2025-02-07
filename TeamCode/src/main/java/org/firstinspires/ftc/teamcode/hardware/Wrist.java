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
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0);
            return false;
        }
    }

    public Action wristDown() {
        return new WristDown();
    }

    public class WristScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.67);
            return false;
        }
    }

    public Action wristScore() {
        return new WristScore();
    }

    public class WristGrabSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.76);
            return false;
        }
    }

    public Action wristGrabSample() {
        return new WristGrabSample();
    }

    public class WristScore2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.95);
            return false;
        }
    }

    public Action wristScore2() {
        return new WristScore2();
    }

    public class Wrist1danger implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(1);
            return false;
        }
    }

    public Action wristDanger() {
        return new Wrist1danger();
    }

    public class WristGrab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.51);
            return false;
        }
    }

    public Action wristGrab() {
        return new WristGrab();
    }

    public class WristMid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.5);
            return false;
        }
    }

    public Action wristMid() {
        return new WristMid();
    }

    public class WristNewScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.3);
            return false;
        }
    }

    public Action wristNewScore() {
        return new WristNewScore();
    }
}
