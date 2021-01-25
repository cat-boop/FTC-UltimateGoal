package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "HsvRingDetectorTest")
public class UGContourRingPipelineJavaExample extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        UGContourRingPipeline.Config.setLowerOrange(new Scalar(75, 130, 50));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar(230, 200, 115));

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        while (opModeIsActive()) {
            String height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.addData("lower orange", UGContourRingPipeline.Config.getLowerOrange());
            telemetry.addData("upper orange", UGContourRingPipeline.Config.getUpperOrange());
            telemetry.update();
        }
    }

//    public static class UGContourRingPipeline extends OpenCvPipeline {
//        /** variable to store the calculated height of the stack **/
//
//        Telemetry telemetry;
//        boolean debug = false;
//
//        public UGContourRingPipeline(Telemetry telemetry, boolean debug) {
//            this.telemetry = telemetry;
//            this.debug = DEBUG;
//
//            this.height = Height.ZERO;
//            this.ret = new Mat();
//            this.mat = new Mat();
//        }
//        /** variables that will be reused for calculations **/
//        Mat mat;
//        Mat ret;
//
//        /** enum class for Height of the stone **/
//        public enum Height {
//            ZERO,
//            ONE,
//            FOUR
//        }
//
//        Height height;
//        /** companion object to store all static variables needed **/
//        public static final class Config {
//
//
//            /** values used for inRange calculation
//             * set to var in-case user wants to use their own tuned values
//             * stored in YCrCb format **/
//            static Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
//            static Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);
//
//            /** width of the camera in use, defaulted to 320 as that is most common in examples **/
//            static int CAMERA_WIDTH = 320;
//
//            /** Horizon value in use, anything above this value (less than the value) since
//             * (0, 0) is the top left of the camera frame **/
//            static int HORIZON = (int) ( (double) (100.0 / 320.0) * CAMERA_WIDTH);
//
//            /** algorithmically calculated minimum width for width check based on camera width **/
//            static double MIN_WIDTH = (50.0 / 320.0) * CAMERA_WIDTH;
//
//            /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
//            static public final double BOUND_RATIO = 0.7;
//
//            public static void setCAMERA_WIDTH(int cameraWidth) {
//                 CAMERA_WIDTH = cameraWidth;
//            }
//
//            public static void setHORIZON(int horizon) {
//                HORIZON = horizon;
//            }
//
//            public static void setLowerOrange(Scalar scalar) {
//                lowerOrange = scalar;
//            }
//
//            public static void setUpperOrange(Scalar scalar) {
//                upperOrange = scalar;
//            }
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            ret.release(); // releasing mat to release backing buffer
//            // must release at the start of function since this is the variable being returned
//
//            ret = new Mat(); // resetting pointer held in ret
//            try { // try catch in order for opMode to not crash and force a restart
//                /**converting from RGB color space to YCrCb color space**/
//                Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//
//                /**checking if any pixel is within the orange bounds to make a black and white mask**/
//                Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
//                Core.inRange(mat, Config.lowerOrange, Config.upperOrange, mask);
//
//                /**applying to input and putting it on ret in black or yellow**/
//                Core.bitwise_and(input, input, ret, mask);
//
//                /**applying GaussianBlur to reduce noise when finding contours**/
//                Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);
//
//                /**finding contours on mask**/
//                List<MatOfPoint> contours = new ArrayList();
//                Mat hierarchy = new Mat();
//                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//
//                /**drawing contours to ret in green**/
//                Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);
//
//                /**finding widths of each contour, comparing, and storing the widest**/
//                int maxWidth = 0;
//                Rect maxRect = new Rect();
//                for (MatOfPoint c : contours) {
//                    Mat copy = new MatOfPoint2f(c);
//                    Rect rect = Imgproc.boundingRect(copy);
//
//                    int w = rect.width;
//                    // checking if the rectangle is below the horizon
//                    if (w > maxWidth && rect.y + rect.height > HORIZON) {
//                        maxWidth = w;
//                        maxRect = rect;
//                    }
//                    c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
//                    copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
//                }
//
//                /**drawing widest bounding rectangle to ret in blue**/
//                Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 2);
//
//                /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
//                Imgproc.line(
//                        ret,
//                        new Point(
//                                .0,
//                                (double) HORIZON
//                        ),
//                        new Point(
//                                (double) CAMERA_WIDTH,
//                                (double) HORIZON
//                        ),
//                        new Scalar(
//                                255.0,
//                                .0,
//                                255.0)
//                );
//
//                if (debug) telemetry.addData("Vision: maxW", maxWidth);
//
//                /** checking if widest width is greater than equal to minimum width
//                 * using Kotlin if expression (Java ternary) to set height variable
//                 *
//                 * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
//                 **/
//                if (maxWidth >= Config.MIN_WIDTH) {
//                    double aspectRatio = (double) maxRect.height / (double) maxRect.width;
//
//                    if(debug) telemetry.addData("Vision: Aspect Ratio", aspectRatio);
//
//                    /** checks if aspectRatio is greater than BOUND_RATIO
//                     * to determine whether stack is ONE or FOUR
//                     */
//                    if (aspectRatio > Config.BOUND_RATIO) height = Height.FOUR; // height variable is now FOUR
//                    else height = Height.ONE; // height variable is now ONE
//                }
//                else {
//                    height = Height.ZERO; // height variable is now ZERO
//                }
//
//                if (debug) telemetry.addData("Vision: Height", height);
//
//                // releasing all mats after use
//                mat.release();
//                mask.release();
//                hierarchy.release();
//
//            } catch (Exception e) {
//                /**error handling, prints stack trace for specific debug**/
//                telemetry.addData("[ERROR]", e);
//            }
//            telemetry.update();
//
//            /**returns the black and orange mask with contours drawn to see logic in action**/
//            return ret;
//        }
//
//        public Height getHeight() {
//            return height;
//        }
//    }

}