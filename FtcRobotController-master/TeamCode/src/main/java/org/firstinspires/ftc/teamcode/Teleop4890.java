package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        telemetry.update();

        robot.init(hardwareMap);

        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            //controller 1 functions
            double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            robot.frontLeft.setPower(FrontLeftVal);
            robot.frontRight.setPower(FrontRightVal);
            robot.backLeft.setPower(BackLeftVal);
            robot.backRight.setPower(BackRightVal);

            if (gamepad1.left_trigger != 0) {
                strafeLeft();
            } else {
                robot.frontRight.setPower(0);
                robot.frontLeft.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
            }

            if (gamepad1.right_trigger != 0) {
                strafeRight();
            } else {
                robot.frontRight.setPower(0);
                robot.frontLeft.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
            }

            //clockwise carousel
            if (gamepad1.a) {
                robot.carousel.setPower(1);
            } else {
                robot.carousel.setPower(0);
            }

            //counter-clockwise carousel
            if (gamepad1.y) {
                robot.carousel.setPower(-1);
            } else {
                robot.carousel.setPower(0);
            }

            //Pivot up forward
            if (gamepad2.right_trigger != 0) {
                robot.pivot.setPower(gamepad2.right_trigger);
            } else {
                robot.pivot.setPower(0);
            }

            //Pivot up backward
            if (gamepad2.left_trigger != 0) {
                robot.pivot.setPower(-gamepad2.left_trigger);
            } else {
                robot.pivot.setPower(0);
            }

            //Arm mover
            if (gamepad2.left_stick_y != 0) {
                robot.arm.setPower(-gamepad2.left_stick_y);
            } else {
                robot.arm.setPower(0);
            }

            //Claw mover
            if (gamepad2.right_stick_y != 0) {
                robot.claw.setPower(gamepad2.right_stick_y);
            } else {
                robot.claw.setPower(0);
            }

            //claw intake
            if (gamepad2.a) {
                robot.clawLeft.setPower(1);
                robot.clawRight.setPower(-1);
            }

            //claw outtake
            if (gamepad2.y) {
                robot.clawLeft.setPower(-1);
                robot.clawRight.setPower(1);
            }

            //claw off
            if (gamepad2.b) {
                robot.clawLeft.setPower(0);
                robot.clawRight.setPower(0);
            }

            idle();
        }
    }


    void strafeLeft() {
        robot.frontLeft.setPower(gamepad1.left_trigger);
        robot.frontRight.setPower(-gamepad1.left_trigger);
        robot.backLeft.setPower(-gamepad1.left_trigger);
        robot.backRight.setPower(gamepad1.left_trigger);
    }

    void strafeRight() {
        robot.frontLeft.setPower(-gamepad1.right_trigger);
        robot.frontRight.setPower(gamepad1.right_trigger);
        robot.backLeft.setPower(gamepad1.right_trigger);
        robot.backRight.setPower(-gamepad1.right_trigger);
    }
}
