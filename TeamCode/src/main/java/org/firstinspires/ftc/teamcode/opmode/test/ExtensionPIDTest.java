package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@Config
@TeleOp
public class ExtensionPIDTest extends LinearOpMode {
    public PivotSubsystem pivot;
    public ExtensionSubsystem extension;
    public static double targetInches = 0;
    public static double targetDegrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        pivot = new PivotSubsystem(hardwareMap);
        extension = new ExtensionSubsystem(hardwareMap, pivot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (!isStopRequested()) {
            extension.periodic();
            pivot.periodic();
            extension.extendInches(targetInches);
            pivot.tiltToPos(targetDegrees);

            telemetry.addData("current pos", extension.getCurrentPosition());
            telemetry.addData("target", targetInches);
            telemetry.addData("current inches", extension.getCurrentInches());
            telemetry.addData("is there", extension.isClose(targetInches));
            telemetry.update();
        }
    }
}