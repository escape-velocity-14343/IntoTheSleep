package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    // wrist constants
    public static double foldedPos = 0.87;
    public static double halfFoldPos = 0.6;
    public static double groundPos = 0.47;
    public static double scoringPos = 0.7;
    public static double specimenScoringPos = 0.7;
    public static double scoringPosReversed = 0.75;
    public static double parkPos = 0.45;
    public static double bucketRetractPos = 0.05;
    public static double specimenReadyPos = 0.33;
    public static double intakeReadyPos = 0.87;

    // wrist command constants
    public static double timeMultiplier = 0.3;

    // claw constants
    // fronttake
    public static double openPos = 0.6;
    public static double closedPos = 0.75;
    public static double singleIntakePos = 0.7;

    public static double clawOffset = -0.16;


    // backtake
    public static double backPos = 0.275;
    public static double backSinglePos = 0.17;
    public static double backClosedPos = 0.12;

    // sub clear constants
    public static double subClearPos = 0.4;
    public static double subClearRetractPos = 0.98;
    public static double subClearMillis = 400;

    // auto constants
    public static double autoOuttakeSpeed = -0.15;
    public static double autoIntakeSpeed = 1;

    // auto heading alignment
    public static double autoAlignP = -0.3;
    public static double autoAlignTol = 10;

    // global offset
    // 1 tick of skip = 0.05 position
    public static double wristOffset = 0.075;

    public static double visionSizeWeight = 1;
    public static double intakeSensorVoltageThres = 0.25;
}
