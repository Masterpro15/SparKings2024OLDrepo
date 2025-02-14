package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final int ARM_COLLAPSED_INTO_ROBOT  = (int)(0*ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN        = (int)(60 * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN2        = (int)(30 * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN3        = (int)(25 * ARM_TICKS_PER_DEGREE);

    final int ARM_SCORE_SAMPLE_IN_LOW   = (int)(90 * ARM_TICKS_PER_DEGREE);

    private DcMotorEx armMotor;
    private OpMode myOpMode;
    public Arm(OpMode opmode) {
        myOpMode = opmode;
    }
    int armposition;
    public void init(){
        armMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class ArmDown implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            armMotor.setTargetPosition(0);
            armMotor.setPower(1);// (adjust speed for whatever is necessary)
            //(you cannot stop motion within this action, you would have to do it in another)
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action armDown() {
        return new ArmDown();
    }
    public class ArmUp implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            armMotor.setTargetPosition(ARM_SCORE_SPECIMEN);
            armMotor.setPower(0.7);// (adjust speed for whatever is necessary)
            //(you cannot stop motion within this action, you would have to do it in another)
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action armUp() {
        return new ArmUp();
    }
    public class ArmStop implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            armMotor.setPower(0);
            return false;
        }
    }
    public Action armStop(){
        return new ArmStop();
    }
    public class ArmSpec2 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setTargetPosition(ARM_SCORE_SPECIMEN2);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action armSpec2(){
        return new ArmSpec2();
    }
    public class ArmSpec3 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setTargetPosition(ARM_SCORE_SPECIMEN2);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action armSpec3(){
        return new ArmSpec3();
    }
    public class ArmBasket implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            armMotor.setTargetPosition(ARM_SCORE_SAMPLE_IN_LOW);
            armMotor.setPower(0.7);// (adjust speed for whatever is necessary)
            //(you cannot stop motion within this action, you would have to do it in another)
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action armBasket() {
        return new ArmBasket();
    }





    public class Armnewspec implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            armMotor.setTargetPosition(770);
            armMotor.setPower(0.7);// (adjust speed for whatever is necessary)
            //(you cannot stop motion within this action, you would have to do it in another)
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action armNewSpec() {
        return new Armnewspec();
    }


    public class Armnewspec2 implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            // desired position, actions do not have parameters (you will have to create a new action for each position you need to go to in auto)
            armMotor.setTargetPosition(425);
            armMotor.setPower(0.7);// (adjust speed for whatever is necessary)
            //(you cannot stop motion within this action, you would have to do it in another)
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action armNewSpec2() {
        return new Armnewspec();
    }


}
