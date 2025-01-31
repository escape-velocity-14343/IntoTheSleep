package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.vision.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.vision.ColorRange;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;
import org.firstinspires.ftc.teamcode.vision.ImageRegion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

@Config
public class CameraSubsystem extends SubsystemBase {
    public static Scalar minimumRed1 = new Scalar(0, 125, 50);
    public static Scalar maximumRed1 = new Scalar(12, 255, 255);

    public static Scalar minimumRed2 = new Scalar(168, 125, 50);
    public static Scalar maximumRed2 = new Scalar(180, 255, 255);

    public static Scalar minimumBlue = new Scalar(100, 125, 50);
    public static Scalar maximumBlue = new Scalar(140, 255, 255);

    public static Scalar minimumYellow = new Scalar(13, 60, 60);
    public static Scalar maximumYellow = new Scalar(50, 255, 255);

    public static double widthMultiplier = 0;

    ColorBlobLocatorProcessorMulti colorLocator;
    ColorSensorProcessor colorSensor;
    VisionPortal portal;
    private double pixelPos = 0;
    private boolean yellow = false;
    public static int exposureMillis = 50;
    public static int minContourArea = 200;
    ColorSensorProcessor.ColorType detection = ColorSensorProcessor.ColorType.NONE;


    public CameraSubsystem(HardwareMap hardwareMap, BooleanSupplier sensorSupplier) {
        colorLocator = new ColorBlobLocatorProcessorMulti(
                new ColorRange(ColorSpace.HSV, new Scalar(13, 60, 60), new Scalar(50, 255, 255)),
                ImageRegion.asImageCoordinates(40, 170, 310, 200),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                -1,
                -1,
                false,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );

        // add yellow colors (same for all alliances)
        colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumYellow, maximumYellow));

        switch (AutoConstants.alliance) {
            case RED:
                colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumRed1, maximumRed1));
                colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumRed2, maximumRed2));
                break;
            case BLUE:
                colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumBlue, maximumBlue));
                break;
        }

        colorSensor = new ColorSensorProcessor(new Rect(new Point(120, 25), new Point(200, 105)), sensorSupplier);
        CameraName camera = hardwareMap.get(WebcamName.class, "rizz");

        portal = new VisionPortal.Builder()
                .addProcessors(colorLocator, colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(camera)
                .enableLiveView(true)
                .build();

        setEnabled(true);
        waitForSetExposure(1000, 1000);
    }

    @Override
    public void periodic() {

        pixelPos = 0;
        yellow = false;

        if (portal.getProcessorEnabled(colorLocator)) {

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            if (blobs.isEmpty()){
                Log.i("%Empty blob list!!", "Uh oh please dont crash");
                return;
            }

            ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, 20000, blobs);
            int dist = 10000;
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            if (!blobs.isEmpty()) {
                /*for (int i = 0; i < Math.min(blobs.size(), 3); i++) {
                    if (Math.abs(160 - blobs.get(i).getBoxFit().center.x) < Math.abs(dist)) {
                        dist = (int) (160 - blobs.get(i).getBoxFit().center.x);
                    }
                }
                pixelPos = dist;*/
                pixelPos = (int) (160 - blobs.get(0).getBoxFit().center.x + (blobs.get(0).getBoxFit().size.width * widthMultiplier / 2.0));
            }
        }
        if (portal.getProcessorEnabled(colorSensor)) {
            detection = colorSensor.getDetection(); //I don't think this can be null so no crashy crashy?
            yellow = detection == ColorSensorProcessor.ColorType.YELLOW;
            Log.v("camera", "color: " + detection);
            // Log.i("camera", "yellow: " + yellow);
        }
    }

    public void setEnabled(boolean enable) {
        portal.setProcessorEnabled(colorLocator, enable);
        portal.setProcessorEnabled(colorSensor, enable);
    }

    public double getPixelPos() {
        return pixelPos;
    }

    public boolean isYellow() {
        return yellow;
    }

    public ColorSensorProcessor.ColorType getColor() {
        return detection;
    }

    /**
     * @return whether the set was successful or not
     */
    public boolean setExposure(int exposure) {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl control = portal.getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return control.setExposure(exposure, TimeUnit.MILLISECONDS);
    }
    public boolean setExposure() {
        return setExposure(exposureMillis);
    }
    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }

    public void setOnlyYellow(boolean onlyYellow) {
        colorLocator.onlyFirstColor = onlyYellow;
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i("camera", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }
    public void saveFrame(String name) {
        portal.saveNextFrameRaw(name);
    }
}
