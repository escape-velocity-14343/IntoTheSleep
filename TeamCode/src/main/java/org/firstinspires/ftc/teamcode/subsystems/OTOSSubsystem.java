package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Localizer;

public class OTOSSubsystem extends SubsystemBase implements Localizer {
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();

    @Override
    public void periodic() {
        pose = otos.getPosition();

    }

    public OTOSSubsystem(String name, HardwareMap hMap) {
        otos = hMap.get(SparkFunOTOS.class, name);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.calibrateImu();
    }

    public SparkFunOTOS.Pose2D getOTOSPose() {
        return pose;
    }

    public Pose2d getPose() {
        return new Pose2d(getOTOSPose().x, getOTOSPose().y, new Rotation2d(getOTOSPose().h));
    }

    public void reset() {
        otos.resetTracking();
    }

    public void setPosition(double x, double y) {
        otos.setPosition(new SparkFunOTOS.Pose2D(x, y, pose.h));
    }
}