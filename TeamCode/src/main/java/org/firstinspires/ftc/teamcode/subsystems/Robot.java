package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;

import java.util.List;

public abstract class Robot extends LinearOpMode {
    List<LynxModule> hubs;
    public ExtensionSubsystem extension;
    public MecanumDriveSubsystem mecanum;
    IMULocalizer imu;
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        mecanum = new MecanumDriveSubsystem("frontRight", "frontLeft", "backRight", "backLeft", hardwareMap, imu);
        extension = new ExtensionSubsystem(hardwareMap);
        imu = new IMULocalizer();
        CommandScheduler.getInstance().registerSubsystem(extension, mecanum, imu);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
