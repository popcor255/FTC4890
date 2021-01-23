package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    double outtakeSpeed = 0;
    double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
    double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
    double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
    double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

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

            if (gamepad1.dpad_down) {
                Orientation angles;
                boolean bStillTurning = true;
                while (bStillTurning) {
                    angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    if (angles.firstAngle > 175 || angles.firstAngle < -175) {
                        bStillTurning = false;
                    }
                    rotate(0.25);

                    idle();
                }
                robot.driveFrontLeft.setPower(0);
                robot.driveBackLeft.setPower(0);
                robot.driveFrontRight.setPower(0);
                robot.driveBackRight.setPower(0);
                sleep(250);
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
                outtakeSpeed = 0.9; //fast
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
                robot.platformRight.setPosition(0.25);
                robot.platformLeft.setPosition(0.75);
            }
            if (gamepad2.dpad_down) { //angles it down
                robot.platformRight.setPosition(0);
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

    //TeleOp Methods (excluding launcher controls)

    //Controller 1 Controls:
    //controls for motors on left side
    void leftDrive() {
        robot.driveFrontLeft.setPower(gamepad1.left_stick_y);
        robot.driveBackLeft.setPower(gamepad1.left_stick_y);
    }

    //controls for motors on right side
    void rightDrive() {
        robot.driveFrontRight.setPower(gamepad1.right_stick_y);
        robot.driveBackRight.setPower(gamepad1.right_stick_y);

    }

    //controls for strafing left
    void strafeLeft() {
        robot.driveFrontRight.setPower(-gamepad1.left_trigger);
        robot.driveFrontLeft.setPower(gamepad1.left_trigger);
        robot.driveBackRight.setPower(gamepad1.left_trigger);
        robot.driveBackLeft.setPower(-gamepad1.left_trigger);
    }

    //controls for strafing right
    void strafeRight() {
        robot.driveFrontRight.setPower(gamepad1.right_trigger);
        robot.driveFrontLeft.setPower(-gamepad1.right_trigger);
        robot.driveBackRight.setPower(-gamepad1.right_trigger);
        robot.driveBackLeft.setPower(gamepad1.right_trigger);
    }

    void rotate(double power) { //note: default rotate clockwise
        robot.driveFrontRight.setPower(-power);
        robot.driveFrontLeft.setPower(power);
        robot.driveBackRight.setPower(-power);
        robot.driveBackLeft.setPower(power);
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
        robot.pusher.setPosition(0.5);
        sleep(400);
        robot.pusher.setPosition(1);
    }
}
