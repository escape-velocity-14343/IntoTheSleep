package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.vision.ColorBlobLocatorProcessorMulti;

import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.vision.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSubsystem extends SubsystemBase {

    ColorBlobLocatorProcessorMulti colorLocator;
    ColorSensorProcessor colorSensor;
    VisionPortal portal;
    private double pixelPos = 0;
    private boolean yellow = false;
    public static int exposureMillis = 60;
    public static int minContourArea = 200;
    ColorSensorProcessor.ColorType detection = ColorSensorProcessor.ColorType.NONE;


    public CameraSubsystem(HardwareMap hardwareMap) {
        /*colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new org.firstinspires.ftc.vision.opencv.ColorRange(ColorSpace.HSV, new Scalar(13, 60, 100), new Scalar(50, 255, 255))) // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blob
                .setRoi(ImageRegion.asImageCoordinates(40,170,310,200))
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .setErodeSize(3)
                .build();*/
        colorLocator = new ColorBlobLocatorProcessorMulti(
                new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, new Scalar (13, 60, 100), new Scalar(50, 255, 255)),
                org.firstinspires.ftc.teamcode.vision.ImageRegion.asImageCoordinates(40, 170,310, 200),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                -1,
                -1,
                false,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
        colorLocator.setColors(
                new ColorRange(ColorSpace.HSV, new Scalar (13, 60, 100), new Scalar(50, 255, 255)),
                new ColorRange(ColorSpace.HSV, new Scalar (70, 150, 100), new Scalar(120, 255, 255)));
        /*colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(130,0,230, 160))
                .setSwatches(PredominantColorProcessor.Swatch.BLUE, PredominantColorProcessor.Swatch.YELLOW, PredominantColorProcessor.Swatch.RED, PredominantColorProcessor.Swatch.WHITE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.GREEN)
                .build();*/
        colorSensor = new ColorSensorProcessor(new Rect(new Point(145, 0), new Point(230, 80)));
        CameraName camera = hardwareMap.get(WebcamName.class, "rizz");

        portal = new VisionPortal.Builder()
                .addProcessors(colorLocator, colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(camera)
                .enableLiveView(true)
                .build();
        setEnabled(true);


    }

    @Override
    public void periodic() {

        pixelPos = 0;
        yellow = false;

        if (portal.getProcessorEnabled(colorLocator)) {

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, 20000, blobs);
            int dist = 10000;
            if (!blobs.isEmpty()) {
                for (int i = 0; i < Math.min(blobs.size(),10); i++) {
                    if (Math.abs(160-blobs.get(i).getBoxFit().center.x)<Math.abs(dist)) {
                        dist = (int) (160-blobs.get(i).getBoxFit().center.x);
                    }
                }
                pixelPos = dist;
            }
        }
        if (portal.getProcessorEnabled(colorSensor)) {
            detection = colorSensor.getDetection();
            yellow = detection == ColorSensorProcessor.ColorType.YELLOW;
            //Log.i("camera", "color: " + detection);
            //Log.i("camera", "yellow: " + yellow);
        }
    }

    public void setEnabled(boolean enable) {
        portal.setProcessorEnabled(colorLocator, enable);
        portal.setProcessorEnabled(colorSensor, enable);
    }
    public double getPixelPos() {
        return pixelPos;
    }
    public boolean getYellow() {
        return yellow;
    }
    public ColorSensorProcessor.ColorType getColor() {
        return detection;
    }
    public boolean setExposure() {
        if (portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl control = portal.getCameraControl(ExposureControl.class);
            control.setMode(ExposureControl.Mode.Manual);
            Log.i("camera", "expousre: " + control.getExposure(TimeUnit.MILLISECONDS));
            control.setExposure(exposureMillis , TimeUnit.MILLISECONDS);

            return true;
        }
        return false;
    }


}
