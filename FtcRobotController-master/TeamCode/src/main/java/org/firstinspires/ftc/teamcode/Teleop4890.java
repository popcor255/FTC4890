package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    double outtakeSpeed = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        telemetry.update();

        robot.init(hardwareMap);

        robot.Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            robot.driveFrontLeft.setPower(FrontLeftVal);
            robot.driveFrontRight.setPower(FrontRightVal);
            robot.driveBackLeft.setPower(BackLeftVal);
            robot.driveBackRight.setPower(BackRightVal);

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

            //intake toggles
            if (gamepad1.right_bumper) { //on (intakes)
                robot.Intake.setPower(1);
            }
            if (gamepad1.left_bumper) { //reverses outtake
                robot.Intake.setPower(-1);
            }
            if (gamepad1.y) {
                robot.Intake.setPower(0); //off
            }

            //outtake toggles
            if (gamepad1.a) {
                outtakeSpeed = 0; //off
            }
            if (gamepad1.x) {
                outtakeSpeed = 1; //fast
            }
            if (gamepad1.b) {
                outtakeSpeed = 0.8; //power shot
            }
            outtakeSys();

            //controller 2 functions
            //arm system for claw
            if (gamepad2.left_stick_y != 0) {
                armSys();
            } else {
                robot.Arm.setPower(0);
            }

            //outtake pusher
            if (gamepad2.a) {
                outtakePush();
            }

            //platform launcher controls
            if (gamepad2.dpad_up) { //angles it upward
                robot.platformRight.setPosition(0.79);
                robot.platformLeft.setPosition(0.81);
            }
            if (gamepad2.dpad_down) { //angles it down
                robot.platformRight.setPosition(0.6);
                robot.platformLeft.setPosition(1);
            }

            //claw functions
            if (gamepad2.y) { //grabs
                robot.grab();
            }
            if (gamepad2.b) { //releases
                robot.release();
            }
            idle();
        }
    }

    //outtake system
    void outtakeSys() {
        robot.outtakeLeft.setPower(outtakeSpeed);
    }

    //Controller 2 Controls:
    //arm controls
    void armSys() {
        robot.Arm.setPower(gamepad2.left_stick_y * 0.5);
    }

    //ring pusher into outtake
    //moves to pushing pos for 0.5 seconds before going back to starting pos.
    void outtakePush() {
        robot.pusher.setPosition(0.8);
        sleep(400);
        robot.pusher.setPosition(1);
    }

    void strafeLeft() {
        robot.driveFrontRight.setPower(gamepad1.left_trigger);
        robot.driveFrontLeft.setPower(-gamepad1.left_trigger);
        robot.driveBackRight.setPower(-gamepad1.left_trigger);
        robot.driveBackLeft.setPower(gamepad1.left_trigger);
    }

    void strafeRight() {
        robot.driveFrontRight.setPower(-gamepad1.right_trigger);
        robot.driveFrontLeft.setPower(gamepad1.right_trigger);
        robot.driveBackRight.setPower(gamepad1.right_trigger);
        robot.driveBackLeft.setPower(-gamepad1.right_trigger);
    }
}