package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
@Config
@TeleOp
public class ExtensionPIDTest extends LinearOpMode {
    public static double targetInches = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        waitForStart();
        while (!isStopRequested()) {
            extensionSubsystem.periodic();
            extensionSubsystem.extendInches(targetInches);

            telemetry.addData("current pos", extensionSubsystem.getCurrentPosition());
            telemetry.addData("target", targetInches);
            telemetry.addData("current inches", extensionSubsystem.getCurrentInches());
            telemetry.addData("is there", extensionSubsystem.isClose(targetInches));
            telemetry.update();
        }
    }
}
