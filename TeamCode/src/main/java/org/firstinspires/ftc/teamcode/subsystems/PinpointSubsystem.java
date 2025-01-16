package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.Localizer;
import org.firstinspires.ftc.teamcode.lib.drivers.GoBildaPinpoint;

import java.util.LinkedList;

@Config

public class PinpointSubsystem extends SubsystemBase implements Localizer {
    GoBildaPinpoint pinpoint;
    Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    @Deprecated
    public static double yawScalar = 1;
    public static boolean flipX = true;
    public static boolean flipY = true;
    public static double xEncOffset = 139.7;
    public static double yEncOffset = 88.9;

    private Pose2D lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public PinpointSubsystem(HardwareMap hMap) {
        pinpoint = hMap.get(GoBildaPinpoint.class, "pinpoint");

        pinpoint.initialize();

        //pinpoint.setYawScalar(yawScalar);
        pinpoint.recalibrateIMU();

        pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                flipX ? GoBildaPinpoint.EncoderDirection.REVERSED : GoBildaPinpoint.EncoderDirection.FORWARD,
                flipY ? GoBildaPinpoint.EncoderDirection.REVERSED : GoBildaPinpoint.EncoderDirection.FORWARD
        );
        pinpoint.setOffsets(xEncOffset, yEncOffset);
        //reset();

    }

    @Override
    public void periodic() {
        pinpoint.update();
        if (Double.isNaN(pinpoint.getPosX())){
            pose = lastGoodPose;
        }
        else{
            pose = pinpoint.getPosition();
            lastGoodPose = pose;
        }

        //Log.i("posex", "" + pose.getX(DistanceUnit.INCH));
        if (DriveConstants.drawRobot) {
            TelemetryPacket packet = new TelemetryPacket();
            drawRobot(packet.field(), getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private Pose2D getSDKPose() {
        return pose;
    }

    public int[] getEncoderCounts() {
        return new int[]{pinpoint.getEncoderX(), pinpoint.getEncoderY()};
    }

    public Pose2d getPose() {
        return new Pose2d(getSDKPose().getX(DistanceUnit.INCH),
                getSDKPose().getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.DEGREES)));
    }

    public Pose2d getVelocity() {
        Pose2D velocity = pinpoint.getVelocity();
        return new Pose2d(velocity.getX(DistanceUnit.INCH),
                velocity.getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(velocity.getHeading(AngleUnit.DEGREES)));
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * @param x In inches.
     * @param y In inches.
     */
    public void setPosition(double x, double y) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
        lastGoodPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }
    public boolean isDoneCalibration() {

        return pinpoint.getDeviceStatus() == GoBildaPinpoint.DeviceStatus.READY;
    }

    public void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 7;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        Vector2d halfv = new Vector2d(t.getRotation().getCos(), t.getRotation().getSin()).times(0.5*ROBOT_RADIUS);
        Vector2d p1 = new Vector2d(t.getX(), t.getY()).plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    /**
     * Warning - will completely break position!!
     */
    public void resetYaw() {
        reset();
    }
}
