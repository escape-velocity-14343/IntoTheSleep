package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline implements VisionProcessor {
    public Scalar lower = new Scalar(0,0,0);
    public Scalar upper = new Scalar(255, 255 ,255);

    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat hierarchy = new Mat();
    private Mat rect = new Mat();
    Point points[] = new Point[4];
    List<MatOfPoint> contours = new ArrayList<>();
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lower, upper, thresholdMat);
        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        /*for (MatOfPoint contour : contours) {
            Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray())).points(points);
            for(int i=0; i<4; ++i){
                Imgproc.line(frame, points[i], points[(i + 1) % 4], new Scalar(255, 255, 255));
            }
        }*/
        Imgproc.drawContours(frame, contours, -1, new Scalar(0,255,0));
        contours.clear();
        thresholdMat.release();
        hsvMat.release();
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
