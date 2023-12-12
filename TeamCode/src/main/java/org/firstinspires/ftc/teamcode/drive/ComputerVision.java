package org.firstinspires.ftc.teamcode.drive;

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
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ComputerVision {

    private final OpenCvWebcam webcam;
    public SpikeMarkPipeline pipeline;


    private static final int STREAM_WIDTH = 960;
    private static final int STREAM_HEIGHT = 720;

    // [[[ax1, ay1], [ax2, ay2]], [...], [...]]
    public static double[][][] points = new double[][][] {
            {{0, 25}, {125, 150}},
            {{380, 10}, {510, 100}},
            {{830, 0}, {960, 120}}
    };

    public static boolean detectRed = true;

    public ComputerVision(HardwareMap hwMap, boolean detectRed, Telemetry telemetry)
    {
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SpikeMarkPipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { }
        });

        ComputerVision.detectRed = detectRed;

        telemetry.addLine("Webcam Init Successful");
        telemetry.update();
    }

    public static class SpikeMarkPipeline extends OpenCvPipeline {
        public SpikeMarkPipeline() { }
        public volatile int location = 0;
        public volatile int p1, p2, p3 = 0;

        @Override
        public Mat processFrame(Mat input) {

            //Create a synchronized block to ensure thread-safe access to the points array
            double[][][] pts; // Local copy of points

            synchronized (ComputerVision.class) {
                pts = points.clone(); // Create a local copy of the points array
            }


            Mat b1 = input.submat(new Rect(new Point(pts[0][0]), new Point(pts[0][1])));
            Mat b2 = input.submat(new Rect(new Point(pts[1][0]), new Point(pts[1][1])));
            Mat b3 = input.submat(new Rect(new Point(pts[2][0]), new Point(pts[2][1])));

            p1 = (int) Core.mean(b1).val[detectRed ? 0 : 2];
            p2 = (int) Core.mean(b2).val[detectRed ? 0 : 2];
            p3 = (int) Core.mean(b3).val[detectRed ? 0 : 2];

            Imgproc.rectangle(
                    input,
                    new Point(pts[0][0]),
                    new Point(pts[0][1]),
                    new Scalar(0, 255, 0),
                    3);

            Imgproc.rectangle(
                    input,
                    new Point(pts[1][0]),
                    new Point(pts[1][1]),
                    new Scalar(225, 255, 0),
                    3);

            Imgproc.rectangle(
                    input,
                    new Point(pts[2][0]),
                    new Point(pts[2][1]),
                    new Scalar(225, 255, 225),
                    3);


            if (p1 > p2 && p1 > p3) {
                location = 1;
            } else if (p2 > p1 && p2 > p3) {
                location = 2;
            } else {
                location = 3;
            }

            return input;
        }
    }
}
