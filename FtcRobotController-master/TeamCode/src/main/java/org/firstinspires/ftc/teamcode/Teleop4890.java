package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {

    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;
    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;
    private DcMotor Intake;
    private DcMotor Arm;
    private Servo Claw;
    private Servo Launcher;

    private ElapsedTime runtime = new ElapsedTime();

    double power = 1;

    //toggles for some of the robot's functions
    boolean intakeToggle = false;
    boolean outtakeToggle = false;

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
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        Intake = hardwareMap.dcMotor.get("Intake");
        Arm = hardwareMap.dcMotor.get("Arm");
        Claw = hardwareMap.servo.get("Claw");
        Launcher = hardwareMap.servo.get("Launcher");

        //sets the direction of motors, right motors always reversed due to their placement
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Claw.setPosition(0);
        Launcher.setPosition(0);

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

            while (intakeToggle = true) {
                intakeSys();
            }

            //controller 2 functions
            //outtake toggle
            //if (gamepad2)

            if (gamepad2.left_stick_y != 0) {
                armSys();
            } else {
                Arm.setPower(0);
            }
            idle();
        }
    }

    //Controller 1 Controls:
    //controls for motors on left side
    void leftDrive() {
        driveFrontLeft.setPower(gamepad1.left_stick_y * power);
        driveBackLeft.setPower(gamepad1.left_stick_y * power);
    }

    //controls for motors on right side
    void rightDrive() {
        driveFrontRight.setPower(gamepad1.right_stick_y * power);
        driveBackRight.setPower(gamepad1.right_stick_y * power);

    }

    //controls for strafing left
    void strafeLeft() {
        driveFrontRight.setPower(gamepad1.left_trigger * power);
        driveFrontLeft.setPower(-gamepad1.left_trigger * power);
        driveBackRight.setPower(-gamepad1.left_trigger * power);
        driveBackLeft.setPower(gamepad1.left_trigger * power);
    }

    //controls for strafing right
    void strafeRight() {
        driveFrontRight.setPower(-gamepad1.right_trigger * power);
        driveFrontLeft.setPower(gamepad1.right_trigger * power);
        driveBackRight.setPower(gamepad1.right_trigger * power);
        driveBackLeft.setPower(-gamepad1.right_trigger * power);
    }

    //intake system
    void intakeSys() {
        Intake.setPower(1);
    }

    //Controller 2 Controls:
    //outtake system
    void outtakeSys() {
        outtakeLeft.setPower(1);
        outtakeRight.setPower(1);
    }

    //arm controls
    void armSys() {
        Arm.setPower(gamepad2.left_stick_y * power);
    }
}


