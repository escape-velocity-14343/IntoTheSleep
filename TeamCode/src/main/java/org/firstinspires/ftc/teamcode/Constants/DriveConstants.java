package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;

@Config
public class DriveConstants {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;

    public static boolean useFieldCentric = false;
    public static boolean drawRobot = false;

    // pto
    public static double ptoRightEngagedPos = 0.5;
    public static double ptoRightDisengagedPos = 0.65;
    public static double ptoRightFreeFloatPos = 0.57;

    public static double ptoLeftEngagedPos = 0.35;
    public static double ptoLeftDisengagedPos = 0.2;
    public static double ptoLeftFreeFloatPos = 0.27;
}
