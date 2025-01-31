package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class AutoConstants {

    public static Pose2d scorePos = new Pose2d(-58, 56, Rotation2d.fromDegrees(-45));

    public static double subBarrierY = 26.5;

    public static double autoscoreMaxVel = 7.5;

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
}
