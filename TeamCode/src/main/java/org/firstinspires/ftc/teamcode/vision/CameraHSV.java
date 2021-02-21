package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.vision.HsvValues.highH;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.highS;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.highV;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowH;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowS;
import static org.firstinspires.ftc.teamcode.vision.HsvValues.lowV;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
public class CameraHSV {


    private OpenCvCamera webcam;
    private UGContourRingPipeline pipeline;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(pipeline = new UGContourRingPipeline(telemetry, true));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(320);

        UGContourRingPipeline.Config.setHORIZON(0);

//        UGContourRingPipeline.Config.setLowerOrange(new Scalar(75, 130, 50));
//        UGContourRingPipeline.Config.setUpperOrange(new Scalar(230, 200, 115));

//        UGContourRingPipeline.Config.setLowerOrange(new Scalar(50, 50, 70));
//        UGContourRingPipeline.Config.setUpperOrange(new Scalar(255, 200, 106));

        UGContourRingPipeline.Config.setLowerOrange(new Scalar(lowH, lowS, lowV));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar(highH, highS, highV));

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    public int getNumberOfRing() {
        UGContourRingPipeline.Height ringPosition = pipeline.getHeight();
        if (ringPosition == UGContourRingPipeline.Height.ZERO) return 0;
        if (ringPosition == UGContourRingPipeline.Height.ONE)  return 1;
        return 4;
    }

    public void stop() {
        webcam.stopStreaming();
    }
}
