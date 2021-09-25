package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp(name = "DirectionTesting")
public class DirectionTesting extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {
                robot.driveFrontLeft.setPower(gamepad1.left_stick_y);
                robot.driveBackLeft.setPower(gamepad1.left_stick_y);
                robot.driveFrontRight.setPower(gamepad1.left_stick_y);
                robot.driveBackRight.setPower(gamepad1.left_stick_y);
            } else {
                robot.driveFrontLeft.setPower(0);
                robot.driveBackLeft.setPower(0);
                robot.driveFrontRight.setPower(0);
                robot.driveBackRight.setPower(0);
            }

            if (gamepad1.right_stick_x != 0) {
                robot.driveFrontLeft.setPower(gamepad1.right_stick_x);
                robot.driveBackLeft.setPower(gamepad1.right_stick_x);
                robot.driveFrontRight.setPower(-gamepad1.right_stick_x);
                robot.driveBackRight.setPower(-gamepad1.right_stick_x);
            } else {
                robot.driveFrontLeft.setPower(0);
                robot.driveBackLeft.setPower(0);
                robot.driveFrontRight.setPower(0);
                robot.driveBackRight.setPower(0);
            }

            if (gamepad1.left_trigger != 0) {
                strafeLeft();
            } else {
                robot.driveFrontLeft.setPower(0);
                robot.driveBackLeft.setPower(0);
                robot.driveFrontRight.setPower(0);
                robot.driveBackRight.setPower(0);
            }

            if (gamepad1.right_trigger != 0) {
                strafeRight();
            } else {
                robot.driveFrontLeft.setPower(0);
                robot.driveBackLeft.setPower(0);
                robot.driveFrontRight.setPower(0);
                robot.driveBackRight.setPower(0);
            }
            idle();
        }
    }

    void strafeLeft() {
        robot.driveFrontRight.setPower(-gamepad1.left_trigger);
        robot.driveFrontLeft.setPower(gamepad1.left_trigger * 0.7);
        robot.driveBackRight.setPower(gamepad1.left_trigger);
        robot.driveBackLeft.setPower(-gamepad1.left_trigger);
    }

    void strafeRight() {
        robot.driveFrontRight.setPower(gamepad1.right_trigger);
        robot.driveFrontLeft.setPower(-gamepad1.right_trigger);
        robot.driveBackRight.setPower(-gamepad1.right_trigger);
        robot.driveBackLeft.setPower(gamepad1.right_trigger);
    }
}