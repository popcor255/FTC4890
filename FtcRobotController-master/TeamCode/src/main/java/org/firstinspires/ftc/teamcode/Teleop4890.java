package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {

    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;
    private DcMotor outtakeLeft;
    private DcMotor Intake;
    private DcMotor Arm;
    private Servo Claw;
    private Servo platformRight;
    private Servo platformLeft;
    private Servo pusher;

    private ElapsedTime runtime = new ElapsedTime();

    double outtakeSpeed = 0;
    boolean intakeToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + runtime.toString());
        telemetry.update();

        //map of the hardware such as the drive motors
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");
        Intake = hardwareMap.dcMotor.get("Intake");
        Arm = hardwareMap.dcMotor.get("Arm");
        Claw = hardwareMap.servo.get("Claw");
        platformRight = hardwareMap.servo.get("platformRight");
        platformLeft = hardwareMap.servo.get("platformLeft");
        pusher = hardwareMap.servo.get("pusher");

        //sets the direction of motors and positions of servos
        //right motors always reversed due to their placement
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        Claw.setPosition(1);
        platformRight.setPosition(0);
        platformLeft.setPosition(1);
        pusher.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            //controller 1 functions
            //left motor drive
            if (gamepad1.left_stick_y != 0) {
                leftDrive();
            } else {
                driveFrontLeft.setPower(0);
                driveBackLeft.setPower(0);
            }

            //right motor drive
            if (gamepad1.right_stick_y != 0) {
                rightDrive();
            } else {
                driveFrontRight.setPower(0);
                driveBackRight.setPower(0);
            }

            //strafe left
            if (gamepad1.left_trigger != 0) {
                strafeLeft();
            } else {
                driveFrontLeft.setPower(0);
                driveBackLeft.setPower(0);
                driveFrontRight.setPower(0);
                driveBackRight.setPower(0);
            }

            //strafe right
            if (gamepad1.right_trigger != 0) {
                strafeRight();
            } else {
                driveFrontLeft.setPower(0);
                driveBackLeft.setPower(0);
                driveFrontRight.setPower(0);
                driveBackRight.setPower(0);
            }

            //intake toggle
            if (gamepad1.right_bumper) {
                intakeToggle = !intakeToggle;
            }
            if (intakeToggle) {
                Intake.setPower(1);
            } else {
                Intake.setPower(0);
            }

            //outtake toggles
            if (gamepad1.a) {
                outtakeSpeed = 0; //off
            }
            if (gamepad1.x) {
                outtakeSpeed = 1; //fast
            }
            if (gamepad1.y) {
                outtakeSpeed = 0.75; //medium
            }
            if (gamepad1.b) {
                outtakeSpeed = 0.50; //slow
            }
            if (gamepad1.left_bumper) {
                outtakeSpeed = 0.60; //power shot
            }
            outtakeSys();

            //controller 2 functions
            //arm system for claw
            if (gamepad2.left_stick_y != 0) {
                armSys();
            } else {
                Arm.setPower(0);
            }

            //outtake pusher
            if (gamepad2.a) {
                outtakePush();
            }

            //platform launcher controls
            if (gamepad2.dpad_up) { //angles it upward
                platformRight.setPosition(0.31);
                platformLeft.setPosition(0.69);
            }
            if (gamepad2.dpad_down) { //angles it down
                platformRight.setPosition(0);
                platformLeft.setPosition(1);
            }

            //claw functions
            if (gamepad2.y) { //grabs
                Claw.setPosition(1);
            }
            if (gamepad2.b) { //releases
                Claw.setPosition(0);
            }
            idle();
        }
    }

    //TeleOp Methods (excluding launcher controls)

    //Controller 1 Controls:
    //controls for motors on left side
    void leftDrive() {
        driveFrontLeft.setPower(gamepad1.left_stick_y);
        driveBackLeft.setPower(gamepad1.left_stick_y);
    }

    //controls for motors on right side
    void rightDrive() {
        driveFrontRight.setPower(gamepad1.right_stick_y);
        driveBackRight.setPower(gamepad1.right_stick_y);

    }

    //controls for strafing left
    void strafeLeft() {
        driveFrontRight.setPower(-gamepad1.left_trigger);
        driveFrontLeft.setPower(gamepad1.left_trigger);
        driveBackRight.setPower(gamepad1.left_trigger);
        driveBackLeft.setPower(-gamepad1.left_trigger);
    }

    //controls for strafing right
    void strafeRight() {
        driveFrontRight.setPower(gamepad1.right_trigger);
        driveFrontLeft.setPower(-gamepad1.right_trigger);
        driveBackRight.setPower(-gamepad1.right_trigger);
        driveBackLeft.setPower(gamepad1.right_trigger);
    }

    //outtake system
    void outtakeSys() {
        outtakeLeft.setPower(outtakeSpeed);
    }

    //Controller 2 Controls:
    //arm controls
    void armSys() {
        Arm.setPower(gamepad2.left_stick_y * 0.5);
    }

    //ring pusher into outtake
    //moves to pushing pos for 0.5 seconds before going back to starting pos.
    void outtakePush() {
        pusher.setPosition(0.5);
        sleep(500);
        pusher.setPosition(1);
    }
}

