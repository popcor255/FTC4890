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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Motor;

public class Robot {

    /* Public OpMode members. */
    public Motor frontRight("frontRight");
    public Motor backRight("backRight");
    public Motor frontLeft("frontLeft");
    public Motor backLeft("backLeft");
    public Motor arm("arm");
    public Motor pivot("pivot");
    public Motor carousel("carousel");
    public Servo claw("claw");
    public Servo clawGrab("clawGrab");
    public DigitalChannel armSensor("armSensor");
    public DigitalChannel clawSensor("clawSensor");

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    public Logger logger = CreateLogger();

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Devices
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        claw = hwMap.crservo.get("claw");
        clawGrab = hwMap.crservo.get("clawGrab");
        arm = hwMap.dcMotor.get("arm");
        pivot = hwMap.dcMotor.get("pivot");
        carousel = hwMap.dcMotor.get("carousel");
        armSensor = hwMap.digitalChannel.get("ArmSensor");
        clawSensor = hwMap.digitalChannel.get("ClawSensor");

        armSensor.setMode(DigitalChannel.Mode.INPUT);
        clawSensor.setMode(DigitalChannel.Mode.INPUT);

        // Setting motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        clawGrab.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private static Logger CreateLogger() {
        Logger logger = Logger.getLogger("MyLogger");
        // don't forward any logging to this logger to his parent
        logger.setUseParentHandlers(false);
        // log messages of all level
        logger.setLevel(Level.ALL);
        // define the logfile
        FileHandler fh = new FileHandler("teleop.log");
        logger.addHandler(fh);
        
        // a Formatter with a custom format
        CustomFormatter formatter = new CustomFormatter();
        fh.setFormatter(formatter);

        return logger;
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void SetPower(DcMotor motor, double power) {
        logger.info(String.format("Motor[%s] || Power[%d]", motor.MotorName, power));
        motor.SetPower(power);
    }
}

