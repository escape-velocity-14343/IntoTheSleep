package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        otos.calibrateImu();
    }

    @Override
    public void periodic() {
        pose = otos.getPosition();
    }

    public SparkFunOTOS.Pose2D getOTOSPose() {
        return pose;
    }

    public Pose2d getPose() {
        return new Pose2d(getOTOSPose().x, getOTOSPose().y, new Rotation2d(Math.toRadians(getOTOSPose().h)));
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
}
