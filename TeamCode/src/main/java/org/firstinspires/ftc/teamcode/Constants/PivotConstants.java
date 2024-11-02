
package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;

    public static double bottomLimit = 3;
    public static double topLimit = 91.5;
    public static double tolerance = 1.5;
    public static double direction = -1;
    public static boolean encoderInvert = true;
    public static double encoderOffset = -128.5;
    public static double outtakeExtendDegrees = 45;
    public static double hangDegrees = 3;
    public static double parkDegrees = 72;
    public static double retractDegrees = 2;

    public static double neutralPos = 3;
}
