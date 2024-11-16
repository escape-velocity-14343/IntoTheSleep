package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

public class CameraSubsystem extends SubsystemBase {

    ColorBlobLocatorProcessor colorLocator;
    PredominantColorProcessor colorSensor;
    VisionPortal portal;
    private double pixelPos = 0;
    private boolean yellow = false;

    public CameraSubsystem(HardwareMap hardwareMap) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(13, 60, 100), new Scalar(50, 255, 255)))         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blob
                .setRoi(ImageRegion.asImageCoordinates(40,170,310,200))
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .setErodeSize(3)
                .build();
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(130,0,230, 160))
                .setSwatches(PredominantColorProcessor.Swatch.BLUE, PredominantColorProcessor.Swatch.YELLOW, PredominantColorProcessor.Swatch.RED, PredominantColorProcessor.Swatch.WHITE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.GREEN)
                .build();
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

            ColorBlobLocatorProcessor.Util.filterByArea(10, 20000, blobs);
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
            yellow = colorSensor.getAnalysis().closestSwatch == PredominantColorProcessor.Swatch.YELLOW;
            Log.i("camera", "yellow: " + yellow);
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


}
