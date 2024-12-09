package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor.ColorType.NONE;
import static org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor.ColorType.YELLOW;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.function.BooleanSupplier;

@Config
public class ColorSensorProcessor implements VisionProcessor {
    Scalar yellowLow = new Scalar(13,60,100);
    Scalar yellowHigh = new Scalar(50,255,255);
    public static double saturationLow = 150;
    public static double valueLow = 50;
    public static double saturationLowBlue = 120;
    public static double valueLowBlue = 50;
    @Deprecated
    public static double redLower = 13;
    public static double colorThreshold = 5;
    Scalar color = new Scalar(0,0,0);
    Rect rect = new Rect();
    Mat subMat = new Mat();
    Mat hsv = new Mat();
    ColorType detection = ColorType.NONE;

    private double red = 0;
    private double yellow = 0;
    private double blue = 0;
    private Mat redMat, redTemp, yellowMat, blueMat;

    private BooleanSupplier hasSample;

    {
        redMat = new Mat();
        redTemp = new Mat();
        yellowMat = new Mat();
        blueMat = new Mat();
    }



    public ColorSensorProcessor(Rect area, BooleanSupplier hasSample) {
        rect = area;
        this.hasSample = hasSample;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Log.v("Voltage Detection", "Detected: " + hasSample.getAsBoolean());
        subMat = frame.submat(rect);
        Imgproc.cvtColor(subMat,hsv,Imgproc.COLOR_RGB2HSV);

        // red
        Core.inRange(hsv, new Scalar(0, saturationLow, valueLow), new Scalar(10, 255, 255), redMat);
        Core.inRange(hsv, new Scalar(150, saturationLow, valueLow), new Scalar(180, 255, 255), redTemp);
        Core.bitwise_or(redMat, redTemp, redMat);
        red = Core.mean(redMat).val[0];

        // yellow
        Core.inRange(hsv, new Scalar(10, saturationLow, valueLow), new Scalar(50, 255, 255), yellowMat);
        yellow = Core.mean(yellowMat).val[0];

        // blue
        Core.inRange(hsv, new Scalar(70, saturationLowBlue, valueLowBlue), new Scalar(120, 255, 255), blueMat);
        blue = Core.mean(blueMat).val[0];

        Log.v("camera color processor", "RYB %: " + red + ", " + yellow + ", " + blue);
        if (red > colorThreshold || (hasSample.getAsBoolean() && red > blue && red > yellow) ) {
            detection = ColorType.RED;
        } else if (blue > colorThreshold || (hasSample.getAsBoolean() && blue > red && blue > yellow)) {
            detection = ColorType.BLUE;
        } else if (yellow > colorThreshold || (hasSample.getAsBoolean() && yellow > blue && yellow > red)) {
            detection = YELLOW;
        } else {

            detection = NONE;
        }

        color = Core.mean(subMat);



        /*Log.i("camera color processor", "HSV: " + color.val[0] + ", " + color.val[1] + ", " + color.val[2]);
        if (color.val[1]>saturationLow && color.val[2]>valueLow) {
            double hue = color.val[0];
            if (hue<0||hue>150 - redLower) {
                detection = ColorType.RED;
            }
            else if (hue>0&&hue<50 - redLower) {
                detection = YELLOW;
            }
            else if (hue>70 - redLower&&hue<120 - redLower) {
                detection = ColorType.BLUE;
            }
            else {
                detection = ColorType.NONE;
            }
        }
        else {
            detection = ColorType.NONE;
        }*/

        return frame;
    }
    public ColorType getDetection() {
        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        int[] colors = {(int) color.val[0], (int) color.val[1], (int) color.val[2]};
        float[] hsvColors = new float[3];
        Color.RGBToHSV(colors[0], colors[1], colors[2], hsvColors);
        rectPaint.setColor(Color.HSVToColor(hsvColors));
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(toGraphicsRect(rect, scaleBmpPxToCanvasPx),rectPaint);
        Paint circlePaint = new Paint();
        switch (detection) {
            case YELLOW: circlePaint.setColor(Color.YELLOW); break;
            case BLUE: circlePaint.setColor(Color.BLUE); break;
            case RED: circlePaint.setColor(Color.RED); break;
            case NONE: circlePaint.setColor(Color.TRANSPARENT); break;

        }
        canvas.drawCircle(Math.round((rect.x+rect.width/2.0)*scaleBmpPxToCanvasPx),Math.round((rect.y+rect.height/2.0)*scaleBmpPxToCanvasPx),scaleCanvasDensity*10, circlePaint);
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
