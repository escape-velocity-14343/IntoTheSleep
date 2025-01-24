package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DrivetrainSquIDController {
    public static double looptimeAdjuster = 15;

    private ElapsedTime loopTime;

    private SquIDController squid;

    public DrivetrainSquIDController() {
        squid = new SquIDController();
        loopTime = new ElapsedTime();
    }

    public void setPID(double p) {
        this.squid.setPID(p);
    }

    public Pose2d calculate(Pose2d targetPose, Pose2d currentPose, Pose2d currentVelocity) {
        double currentVelMag = currentVelocity.getTranslation().getNorm();
        // compensate for pinpoint trolling
        double deltaSeconds = loopTime.seconds();
        currentVelMag *= deltaSeconds;

        double velAngle = Math.atan2(currentVelocity.getY(), currentVelocity.getX());

        double distance = Math.max(getDistanceFromVelocity(currentVelMag), 0);

        currentPose = currentPose.plus(new Transform2d(new Translation2d(Math.cos(velAngle) * distance,
                Math.sin(velAngle) * distance), new Rotation2d()));

        double magnitude = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        magnitude = squid.calculate(magnitude, 0);
        Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
        double posAngle = Math.atan2(delta.getY(), delta.getX());
        loopTime.reset();
        return new Pose2d(Math.cos(posAngle) * magnitude, Math.sin(posAngle) * magnitude, new Rotation2d());
    }

    private static double getDistanceFromVelocity(double velocity) {
        velocity *= looptimeAdjuster;
        // equation from regression
        return 0.00286 * velocity * velocity + 0.304 * velocity - 0.837;
    }

}
