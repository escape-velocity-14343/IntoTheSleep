package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config

public class SlideConstants {
    public static double kP = 0.0045;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double ticksPerInch = 31.6;
    public static double maxExtension = 29.5;

    public static double bucketPos = 28;
    public static double lowBucketPos = 11.5;

    public static double tiltInches = 15;

    public static double minExtension = 0;
    public static double direction = 1;
    public static double tolerance = 1;
    public static double alertCurrent = 4;

    /**
     * The maximum extension, in inches, while pivoting down.
     */
    public static double maxPivotExtension = 25;
    /**
     * Constant feedforward for the slides (probably don't need)
     */
    public static double FEEDFORWARD_STATIC = 0.0;
    /**
     * Feedforward value that is multiplied by <code>Math.cos(slideAngle)</code>
     */
    public static double FEEDFORWARD_DYNAMIC = 0.2;


    public static double autonPiece1Extension = 10;
    public static double autonPiece3Extension = 13.5;

    public static double submersibleIntakeMaxExtension = 17.5;

    public static double autonPiecee3ExtensionPower = 0.3;
    public static double autoRetractionPower = 1.0;

    static public double specimenRaisePosition = 0;
    public static double specimenHighRaisePosition = 13;
    public static double specimenHookPosition = 16.0;

    public static double millisPerInch = 5 * ticksPerInch;
    public static double visionP = 0.5;

    public static double bucketPosGainScheduleMult = 1.5;
    public static double bucketPosGainSchedulePos = 25;
}
