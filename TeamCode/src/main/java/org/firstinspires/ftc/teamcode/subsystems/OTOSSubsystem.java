package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.Localizer;

// Offset from bot: 147.5 mm (y), 46.5 mm (x)
// Offset from bot: 5.8 in (y), 1.83 in (x)
// (just figure it out since the otos is obviously more in one direction from the chassis

public class OTOSSubsystem extends SubsystemBase implements Localizer {
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();

    public OTOSSubsystem(HardwareMap hMap) {
        otos = hMap.get(SparkFunOTOS.class, "otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(-1.3, 5.8, 180));
        otos.setAngularScalar(360/360.4);
        otos.calibrateImu(255, true);
    }

    @Override
    public void periodic() {
        pose = otos.getPosition();
        if (DriveConstants.drawRobot) {
            TelemetryPacket packet = new TelemetryPacket();
            drawRobot(packet.field(), getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public SparkFunOTOS.Pose2D getOTOSPose() {
        return pose;
    }

    public Pose2d getPose() {
        return new Pose2d(getOTOSPose().x, getOTOSPose().y, Rotation2d.fromDegrees(getOTOSPose().h));
    }

    public void reset() {
        otos.resetTracking();
    }

    public void setPosition(double x, double y) {
        otos.setPosition(new SparkFunOTOS.Pose2D(x, y, pose.h));
    }
    public int getCalibrationProgress() {
        return otos.getImuCalibrationProgress();
    }
    public boolean isDoneCalibration() {
        return getCalibrationProgress()<1;
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
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }
}
