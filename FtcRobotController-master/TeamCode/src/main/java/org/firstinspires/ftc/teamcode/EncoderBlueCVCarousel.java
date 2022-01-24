package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "EncoderBlueCVCarousel", preselectTeleOp = "Teleop4890")
public class EncoderBlueCVCarousel extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;
    Robot robot = new Robot();



    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 webcam at 1280x720.
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

//    int ringNumber;
//    boolean detection = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes the camera and sets it up for which camera will be used.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        double position = 0;

        while (!isStarted()) {
            ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        pipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        pipeline.setDecimation(DECIMATION_HIGH);
                    }

                    if ((detections.get(0).pose.x * FEET_PER_METER) < 0) {
                        position = 1;
                    } else if ((detections.get(0).pose.x * FEET_PER_METER) < 2 && (detections.get(0).pose.x * FEET_PER_METER) > 0) {
                        position = 2;
                    } else if ((detections.get(0).pose.x * FEET_PER_METER) > 2) {
                        position = 3;
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }

                telemetry.update();
            }
        }

        sleep(20);

        if (position == 1) {
            // low level
            straight(0.35, 500);
            rotate(0.5, 320);
            straight(0.35, 450);
            while (robot.clawSensor.getState()) {
                robot.setPower(claw, 1);
                // moveClaw(1, 2000);
            }
            robot.setPower(claw, 0);


            robot.pivot.setPower(-0.35);
            sleep(275);
            robot.pivot.setPower(0);
            straight(0.20,400);

            outtake(3000);
            straight(-0.35,300);

            rotate(0.5, 200);
            straight(-0.5,1300);
            carouselClock(3000);

            rotate(-0.5, 540);
            straight(0.35, 400);

            stop();
        } else if (position == 2) {
            // medium level
            straight(0.35, 550);
            rotate(0.5, 300);
            straight(0.35, 700);
            while (robot.clawSensor.getState()) {
                robot.setPower(claw, 1);
                // moveClaw(1, 2000);
            }
            robot.setPower(claw, 0);

            outtake(3000);
            straight(-0.35,300);

            rotate(0.5, 200);
            straight(-0.5,1300);
            carouselClock(3000);

            rotate(-0.5, 540);
            straight(0.35, 400);

            stop();
        } else if (position == 3) {
            straight(0.35, 575);
            rotate(0.5, 300);
            straight(0.35, 450);
            while (robot.clawSensor.getState()) {
                robot.setPower(claw, 1);
                // moveClaw(1, 3000);
            }
            robot.setPower(claw, 0);

//                robot.arm.setPower(0.9);
//                sleep(2200);
//                robot.arm.setPower(0);
            while (robot.armSensor.getState()) {
                robot.arm.setPower(0.9);
            }
            robot.arm.setPower(0);

            outtake(3000);
            robot.arm.setPower(-0.9);
            sleep(1500);
            robot.arm.setPower(0);
            straight(-0.35,300);

            rotate(0.5, 200);
            straight(-0.5,1300);
            carouselClock(3000);

            rotate(-0.5, 540);
            straight(0.35, 400);

            stop();
        }
    }

    void straight(double power, int milliseconds) {
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        sleep(milliseconds);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    void strafeLeft(double power, int milliseconds) {
        robot.frontRight.setPower(-power);
        robot.frontLeft.setPower(power * 0.5);
        robot.backRight.setPower(power);
        robot.backLeft.setPower(-power);
        sleep(milliseconds);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    void strafeRight(double power, int milliseconds) {
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.backRight.setPower(-power);
        robot.backLeft.setPower(power);
        sleep(milliseconds);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    void rotate(double power, int milliseconds) { //note: default rotate counter clockwise
        robot.frontRight.setPower(-power);
        robot.frontLeft.setPower(power);
        robot.backRight.setPower(-power);
        robot.backLeft.setPower(power);
        sleep(milliseconds);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    void moveClaw(double power, int milliseconds) {
        robot.setPower(claw, power);
        sleep(milliseconds);
        robot.setPower(claw, 0);
    }

    void outtake(int milliseconds) {
        robot.setPower(clawGrab, -1);
        sleep((milliseconds));
        robot.setPower(clawGrab, 0);
    }

    void carouselClock(int milliseconds) {
        robot.carousel.setPower(-0.60);
        sleep(milliseconds);
        robot.carousel.setPower(0);
    }
}
