/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    /* Public OpMode members. */
    public DcMotor driveFrontRight;
    public DcMotor driveFrontLeft;
    public DcMotor driveBackRight;
    public DcMotor driveBackLeft;
    public DcMotor outtakeLeft;
    public DcMotor Intake;
    public DcMotor Arm;
    public Servo Claw;
    public Servo platformRight;
    public Servo platformLeft;
    public Servo pusher;
    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Retrieve and initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        driveFrontRight = hwMap.dcMotor.get("driveFrontRight");
        driveFrontLeft = hwMap.dcMotor.get("driveFrontLeft");
        driveBackRight = hwMap.dcMotor.get("driveBackRight");
        driveBackLeft = hwMap.dcMotor.get("driveBackLeft");
        Intake = hwMap.dcMotor.get("Intake");
        outtakeLeft = hwMap.dcMotor.get("outtakeLeft");
        Arm = hwMap.dcMotor.get("Arm");

        // Setting motor directions
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        Intake.setPower(0);
        Arm.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);
        outtakeLeft.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        Claw = hwMap.servo.get("Claw");
        platformRight = hwMap.servo.get("platformRight");
        platformLeft = hwMap.servo.get("platformLeft");
        pusher = hwMap.servo.get("pusher");
        Claw.setPosition(1);
        platformRight.setPosition(0);
        platformLeft.setPosition(1);
        pusher.setPosition(1);
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    void grab(int milliseconds) {
        Claw.setPosition(1);
        sleep(milliseconds);
    }

    void grab() {
        grab(0);
    }

    void release(int milliseconds) {
        Claw.setPosition(0);
        sleep(milliseconds);
    }

    void release() {
        release(0);
    }
}

