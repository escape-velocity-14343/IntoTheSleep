package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class ColorSensorProcessor implements VisionProcessor {
    Scalar yellowLow = new Scalar(13,60,100);
    Scalar yellowHigh = new Scalar(50,255,255);
    public static double saturationLow = 60;
    public static double valueLow = 40;
    Scalar color = new Scalar(0,0,0);
    Rect rect = new Rect();
    Mat subMat = new Mat();
    ColorType detection = ColorType.NONE;



    public ColorSensorProcessor(Rect area) {
        rect = area;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        subMat = frame.submat(rect);
        Imgproc.cvtColor(subMat,subMat,Imgproc.COLOR_RGB2HSV);
        color = Core.mean(subMat);
        if (color.val[1]>saturationLow && color.val[2]>valueLow) {
            double hue = color.val[0];
            if (hue<13||hue>150) {
                detection = ColorType.RED;
            }
            else if (hue>13&&hue<50) {
                detection = ColorType.YELLOW;
            }
            else if (hue>70&&hue<120) {
                detection = ColorType.BLUE;
            }
            else {
                detection = ColorType.NONE;
            }
        }
        else {
            detection = ColorType.NONE;
        }
        return frame;
    }
    public ColorType getDetection() {
        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        float[] colors = {(float) color.val[0], (float) color.val[1], (float) color.val[2]};
        rectPaint.setColor(Color.HSVToColor(colors));
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(toGraphicsRect(rect, scaleBmpPxToCanvasPx),rectPaint);
    }

    private android.graphics.Rect toGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round((rect.x+rect.width)*scaleBmpPxToCanvasPx);
        int bottom = Math.round((rect.y+rect.height)*scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }
    public enum ColorType {
        YELLOW,
        BLUE,
        RED,
        NONE
    }

}
