package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Size;

public class CoarseDetect implements VisionProcessor {
    double fx,fy;
    Size size;
    double x,y,z,pivot; //inches and radians
    Pose2d robotPos;
    Translation2d samplePos = new Translation2d(0,0);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        fx = calibration.focalLengthX;
        fy = calibration.focalLengthY;
        size = new Size(width, height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    //Robot Centric
    public void pixelToPosition(int x, int y) {
        double angle = Math.atan(y/fy);
        double floorDist = z/Math.tan(angle+pivot);
        double trueDist = Math.hypot(floorDist, z);
        double planeDist = Math.cos(angle)*trueDist;
        samplePos = new Translation2d(floorDist, -planeDist*x/fx);
    }

    public Translation2d getPosition() {
        return samplePos;
    }

}
