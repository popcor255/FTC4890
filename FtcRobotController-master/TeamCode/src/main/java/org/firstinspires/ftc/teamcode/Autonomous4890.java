package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name = "Autonomous4890", preselectTeleOp = "Teleop4890")
public class Autonomous4890 extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
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

    int ringNumber;
    boolean detection = true;

    @Override
    public void runOpMode() {

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

        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        outtakeLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        Claw.setPosition(1);
        platformRight.setPosition(0);
        platformLeft.setPosition(1);
        pusher.setPosition(1);

        //initializes the camera and sets it up for which camera will be used.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                phoneCam.setFlashlightEnabled(true);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            while (detection) {
                if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                    ringNumber = 0;
                    detection = false;
                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                    ringNumber = 1;
                    detection = false;
                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                    ringNumber = 4;
                    detection = false;
                }
            }
            if (ringNumber == 0) {
                armDown(1050);
                straight(1, 3310);
                strafeRight(1, 600);
                Release(500);
                strafeLeft(1, 970);
                rotate(-1, 535);
                strafeRight(1, 2770);
                Grab(500);
                strafeLeft(1, 2650);
                rotate(1, 650);
                strafeRight(1, 930);
            } else if (ringNumber == 1) {
                armDown(1050);
                strafeRight(1, 450);
                straight(1, 4100);
                strafeLeft(1, 1200);
                Release(700);
                strafeLeft(1, 1300);
                rotate(-1, 90);
                straight(-1, 3300);
                strafeRight(1, 500);
                Grab(500);
                straight(1, 3600);
                rotate(1, 90);
                strafeRight(1, 1250);
                Release(500);
                strafeLeft(1, 600);
                straight(-1, 500);
            } else if (ringNumber == 4) {
                armDown(1050);
                strafeRight(1, 450);
                straight(1, 5000);
                rotate(1, 300);
                Release(500);
                strafeLeft(1, 3700);
                rotate(-1, 888);
                strafeRight(1, 2250);
                Grab(500);
                strafeLeft(1, 2500);
                straight(1, 3400);
                rotate(1, 270);
                Release(500);
                strafeLeft(1, 400);
                straight(-1, 1500);
            }

            stop();
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        // An enum to define the amount of rings

        public enum RingPosition {
            FOUR, //C (box in back corner)
            ONE,  //B (box not next to wall)
            NONE  //A (box on middle line)
        }

        // Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        // The core values which define the location and size of the sample regions
        // topleft = where the topleft point of the analysis region begins
        // region width & height determines the analysis' dimensions
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(215, 55);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 146;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Working variables
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        // This function takes the RGB frame, converts to YCrCb,
        // and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }

    void straight(double power, int milliseconds) {
        driveFrontLeft.setPower(-power);
        driveBackLeft.setPower(-power);
        driveFrontRight.setPower(-power);
        driveBackRight.setPower(-power);
        sleep(milliseconds);
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);
    }

    void strafeLeft(double power, int milliseconds) {
        driveFrontRight.setPower(-power);
        driveFrontLeft.setPower(power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(-power);
        sleep(milliseconds);
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);
    }

    void strafeRight(double power, int milliseconds) {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(-power);
        driveBackLeft.setPower(power);
        sleep(milliseconds);
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);
    }

    void rotate(double power, int milliseconds) { //note: default rotate clockwise
        driveFrontRight.setPower(-power);
        driveFrontLeft.setPower(power);
        driveBackRight.setPower(-power);
        driveBackLeft.setPower(power);
        sleep(milliseconds);
        driveFrontLeft.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackRight.setPower(0);
    }

    void platformUp(int milliseconds) {
        platformRight.setPosition(0.24);
        platformLeft.setPosition(0.76);
        sleep(milliseconds);
    }

    void platformDown(int milliseconds) {
        platformRight.setPosition(0);
        platformLeft.setPosition(1);
        sleep(milliseconds);
    }

    void outtakePush(int milliseconds) {
        pusher.setPosition(0.5);
        sleep(400);
        pusher.setPosition(1);
        sleep(milliseconds);
    }

    void armUp(int milliseconds) {
        Arm.setPower(0.5);
        sleep(milliseconds);
        Arm.setPower(0);
    }

    void armDown(int milliseconds) {
        Arm.setPower(-0.5);
        sleep(milliseconds);
        Arm.setPower(0);
    }

    void Grab(int milliseconds) {
        Claw.setPosition(1);
        sleep(milliseconds);
    }

    void Release(int milliseconds) {
        Claw.setPosition(0);
        sleep(milliseconds);
    }
}