package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Gyro: OUTREACH ", group="Linear OpMode")

public class OutReachREAL extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    public DcMotor  liftMotor = null;
    public Servo    wrist = null; //the wrist servo
    public Servo    claw  = null;
    private DcMotor LeftHang = null;
    private DcMotor RightHang = null;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT  = 10;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double WRIST_FOLDED_IN   = 0;
    final double WRIST_FOLDED_OUT  = 0.78;
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_COLLECT = 1834;
    final double LIFT_SCORING_IN_HIGH_BASKET = 610 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    double armLiftComp = 0.4;
    final double claw_OPEN = 0;
    final double claw_CLOSE= 1;
    int state = 1;
    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        claw  = hardwareMap.get(Servo.class, "claw");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist  = hardwareMap.get(Servo.class, "wrist");

        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Robot Ready.");
        telemetry.update();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            {  double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (gamepad1.start) {
                    imu.resetYaw();
                }
            double max;
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX = rotX * 1.1;  // Counteract imperfect strafing
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double leftFrontPower = (rotY + rotX + rx) / denominator;
                double leftBackPower = (rotY - rotX + rx) / denominator;
                double rightFrontPower = (rotY - rotX - rx) / denominator;
                double rightBackPower = (rotY + rotX - rx) / denominator;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

                boolean dpadUpPressed = false;

                while (opModeIsActive()) {
                    // Toggle the state when dpad_up is pressed
                    if (gamepad1.dpad_up && !dpadUpPressed) {
                        state *= -1;
                        dpadUpPressed = true;
                    } else if (!gamepad1.dpad_up) {
                        dpadUpPressed = false;
                    }
                    switch(state) {
                        case 1:
                            claw.setPosition(claw_CLOSE);
                            break;
                        case -1:
                            claw.setPosition(claw_OPEN);
                            break;
                    }
                }
                if(gamepad1.b){
                wrist.setPosition(0);
            }
            if(gamepad1.dpad_left){
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            if(gamepad1.dpad_down){
                // change this to pickup wrist.setPosition(0.67);
            }
            if(gamepad1.dpad_right){
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            if(gamepad1.y){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                liftPosition = LIFT_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                armPosition = 0;

            }
            else if (gamepad1.b){
                //actually high basket
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;

            }

            else if (gamepad1.a) {
                claw.setPosition(claw_CLOSE);
                wrist.setPosition(WRIST_FOLDED_IN);
                sleep(250);
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                liftPosition =LIFT_COLLAPSED;
            }
            if (gamepad1.right_trigger > 0.2){
                armPosition = armPosition + 18;
            }
            if (gamepad1.left_trigger >  0.2){
                armPosition = armPosition - 18;
            }
            if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
                armLiftComp = (0.17 * liftPosition);
            }
            else{
                armLiftComp = 0;
            }

            if (gamepad1.right_bumper){
                liftPosition += 2800 * cycletime;
            }
            else if (gamepad1.left_bumper){
                liftPosition -= 2800 * cycletime;
                wrist.setPosition(0);
            }

            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET){
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            if (liftPosition < 0){
                liftPosition = 0;
            }

            liftMotor.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) liftMotor).setVelocity(2100);

            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
            telemetry.addData("lift current position", liftMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("wrist position" , wrist.getPosition());
            telemetry.addData("wrist direction" , wrist.getDirection());
            telemetry.update();

    }}}}
