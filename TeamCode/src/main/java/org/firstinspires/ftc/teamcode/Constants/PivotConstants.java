
package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kP = 0.012;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;

    public static double bottomLimit = 0.5;
    public static double topLimit = 93;
    public static double tolerance = 1.5;
    public static double direction = -1;
    public static boolean encoderInvert = true;
    public static double encoderOffset = 47.7 ;
    public static double outtakeExtendDegrees = 40;
    public static double hangDegrees = -10;
    public static double parkDegrees = 80;
    public static double retractDegrees = bottomLimit;
    public static double reversedRetractDegrees = 10;
    public static double frontOuttakePosition = 85;

    public static double neutralPos = bottomLimit;

    public static double specimenIntakeAngle = topLimit;
    public static double specimenTopBarAngle = 60;
}
