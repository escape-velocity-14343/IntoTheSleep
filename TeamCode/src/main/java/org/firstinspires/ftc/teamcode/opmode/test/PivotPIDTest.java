package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@Config
@TeleOp
public class PivotPIDTest extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PivotSubsystem pivot = new PivotSubsystem(hardwareMap);
        waitForStart();
        while (!isStopRequested()) {
            pivot.periodic();
            pivot.tiltToPos(target);

            telemetry.addData("current pos", pivot.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("is there", pivot.isClose(target));
            telemetry.update();
        }
    }
}
