package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class AutoConstants {

    public static Pose2d scorePos = new Pose2d(-57, 57, Rotation2d.fromDegrees(-45));

    public static double subBarrierY = 24;

    public static double autoscoreMaxVel = 8.5;

    // Milliseconds
    public static long outtakePause = 50;
    public static long outtakeTimeout = 1300;

    public static double autoscoreMaxPivotVel = 1.0;

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
}
