package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.vision.HsvValues.highH;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.highS;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.highV;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowH;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowS;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowV;

@TeleOp
//@Config
public class Hsv extends LinearOpMode {

    //public static int lowH = 0, lowS = 0, lowV = 0;
    //public static int highH = 0, highS = 0, highV = 0;

    OpenCvCamera webcam;
    StackSizeDetector pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new StackSizeDetector(telemetry);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
        }

        webcam.stopStreaming();
    }

    public static class StackSizeDetector extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location {
            LEFT,
            RIGHT,
            NOT_FOUND
        }

        private Location location;

        static final Rect LEFT_ROI = new Rect(
                new Point(60, 35),
                new Point(120, 75));
        static final Rect RIGHT_ROI = new Rect(
                new Point(140, 35),
                new Point(200, 75));
        static double PERCENT_COLOR_THRESHOLD = 0.4;

        public StackSizeDetector(Telemetry t) {
            telemetry = t;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(lowH, lowS, lowV);
            Scalar highHSV = new Scalar(highH, highS, highV);

            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left = mat.submat(LEFT_ROI);
            Mat right = mat.submat(RIGHT_ROI);

            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

            //left.release();
            //right.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

            boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

            if (stoneLeft && stoneRight) {
                location = Location.NOT_FOUND;
                telemetry.addData("Skystone Location", "not found");
            } else if (stoneLeft) {
                location = Location.RIGHT;
                telemetry.addData("Skystone Location", "right");
            } else {
                location = Location.LEFT;
                telemetry.addData("Skystone Location", "left");
            }
            telemetry.update();

            //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar colorStone = new Scalar(255, 0, 0);
            Scalar colorSkystone = new Scalar(0, 255, 0);

            //Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorSkystone : colorStone);
            //Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorSkystone : colorStone);

            return mat;
        }

        public Location getLocation() {
            return location;
        }
    }
}
