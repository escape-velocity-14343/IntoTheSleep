package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class ExtensionPIDTest extends Robot {
    public static double targetInches = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (!isStopRequested()) {
            extension.periodic();
            extension.extendInches(targetInches);

            telemetry.addData("current pos", extension.getCurrentPosition());
            telemetry.addData("target", targetInches);
            telemetry.addData("current inches", extension.getCurrentInches());
            telemetry.addData("is there", extension.isClose(targetInches));
            telemetry.update();
        }
    }
}
