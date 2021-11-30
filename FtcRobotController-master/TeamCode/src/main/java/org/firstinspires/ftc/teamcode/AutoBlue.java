package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "AutoBlue", preselectTeleOp = "Teleop4890")
public class AutoBlue extends LinearOpMode {
//    OpenCvInternalCamera phoneCam;
//    SkystoneDeterminationPipeline pipeline;
    Robot robot = new Robot();

//    int ringNumber;
//    boolean detection = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        //initializes the camera and sets it up for which camera will be used.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//      phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline();
//        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
//
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//                phoneCam.setFlashlightEnabled(true);
//            }
//            @Override
//            public void onError(int errorCode) {
//            }
//        });

        waitForStart();

        while (opModeIsActive()) {

            straight(.5,500);
            rotate(1,250);
            straight(1,800);

            stop();

//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);

//            while (detection) {
//                if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
//                    ringNumber = 0;
//                    detection = false;
//                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
//                    ringNumber = 1;
//                    detection = false;
//                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
//                    ringNumber = 4;
//                    detection = false;
//                }
//            }

//            if (ringNumber == 0) {
//                straight(1, 570);
//                sleep(1000);
//                armDown(750);
//                robot.release(800);
//                strafeLeft(1, 350);
//                sleep(1000);
//                straight(-1, 160);
//                platformUp(100);
//                robot.outtakeLeft.setPower(0.87);
//                sleep(1000);
//                outtakePush(800);
//                outtakePush(800);
//                outtakePush(800);
//                straight(1,120);
//            } else if (ringNumber == 1) {
//                strafeRight(1,525);
//                sleep(750);
//                rotate(0.5,5);
//                sleep(50);
//                straight(1, 840);
//                sleep(1100);
//                strafeLeft(1, 950);
//                sleep(300);
//                armDown(1000);
//                robot.release(800);
//                strafeLeft(1,200);
//                armUp(700);
//                sleep(1000);
//                straight(-1, 300);
//                sleep(400);
//                strafeRight(1,375);
//                //rotate(-0.5, 40);
//                sleep(100);
//                platformUp(100);
//                robot.outtakeLeft.setPower(0.87);
//                sleep(1000);
//                outtakePush(800);
//                outtakePush(800);
//                outtakePush(800);
//                straight(1,100);
//            } else if (ringNumber == 4) {
//                strafeRight(1,400);
//                sleep(750);
//                rotate(1,1);
//                sleep(50);
//                straight(1, 1200);
//                sleep(1000);
//                armDown(1000);
//                robot.release(800);
//                sleep(1000);
//                strafeLeft(1,340);
//                straight(-1, 460);
//                platformUp(100);
//                robot.outtakeLeft.setPower(0.87);
//                sleep(1000);
//                outtakePush(800);
//                outtakePush(800);
//                outtakePush(800);
//                straight(1,100);
//            }
//
//            stop();
        }
    }

//    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
//        // An enum to define the amount of rings
//
//        public enum RingPosition {
//            FOUR, //C (box in back corner)
//            ONE,  //B (box not next to wall)
//            NONE  //A (box on middle line)
//        }
//
//        // Some color constants
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//
//        // The core values which define the location and size of the sample regions
//        // topleft = where the topleft point of the analysis region begins
//        // region width & height determines the analysis' dimensions
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(185, 58);
//
//        static final int REGION_WIDTH = 35;
//        static final int REGION_HEIGHT = 25;
//
//        final int FOUR_RING_THRESHOLD = 160;
//        final int ONE_RING_THRESHOLD = 145;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        // Working variables
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        private volatile RingPosition position = RingPosition.FOUR;
//
//        // This function takes the RGB frame, converts to YCrCb,
//        // and extracts the Cb channel to the 'Cb' variable
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame) {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            position = RingPosition.FOUR; // Record our analysis
//            if (avg1 > FOUR_RING_THRESHOLD) {
//                position = RingPosition.FOUR;
//            } else if (avg1 > ONE_RING_THRESHOLD) {
//                position = RingPosition.ONE;
//            } else {
//                position = RingPosition.NONE;
//            }
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            return input;
//        }
//
//        public int getAnalysis() {
//            return avg1;
//        }
//    }

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

//    void platformUp(int milliseconds) {
//        robot.platformRight.setPosition(0.79);
//        robot.platformLeft.setPosition(0.81);
//        sleep(milliseconds);
//    }

//    void platformDown(int milliseconds) {
//        robot.platformRight.setPosition(0.6);
//        robot.platformLeft.setPosition(1);
//        sleep(milliseconds);
//    }

//    void outtakePush(int milliseconds) {
//        robot.pusher.setPosition(0.85);
//        sleep(400);
//        robot.pusher.setPosition(1);
//        sleep(milliseconds);
//    }

//    void armUp(int milliseconds) {
//        robot.Arm.setPower(-0.5);
//        sleep(milliseconds);
//        robot.Arm.setPower(0);
//    }

//    void armDown(int milliseconds) {
//        robot.Arm.setPower(0.5);
//        sleep(milliseconds);
//        robot.Arm.setPower(0);
//    }
}
