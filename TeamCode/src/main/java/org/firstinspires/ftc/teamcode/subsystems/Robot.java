package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;

import java.util.List;

public abstract class Robot extends LinearOpMode {
    List<LynxModule> hubs;
    public ExtensionSubsystem extension;
    public MecanumDriveSubsystem mecanum;
    public PivotSubsystem pivot;
    public WristSubsystem wrist;
    public IntakeSubsystem intake;
    IMULocalizer imu;
    public OTOSSubsystem otos;
    public ElapsedTime timer = new ElapsedTime();
    public CommandScheduler cs = CommandScheduler.getInstance();
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        otos = new OTOSSubsystem(hardwareMap);
        mecanum = new MecanumDriveSubsystem("frontRight", "frontLeft", "backRight", "backLeft", hardwareMap, otos);
        pivot = new PivotSubsystem(hardwareMap);
        extension = new ExtensionSubsystem(hardwareMap, pivot);
        wrist = new WristSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        imu = new IMULocalizer();

        CommandScheduler.getInstance().registerSubsystem(extension, mecanum, imu, otos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    public Command intakePos() {
        return new IntakePosCommand(extension, pivot, wrist);
    }
    public Command subPos() {
       return new SubPosCommand(extension, wrist, intake);
    }
    public Command bucketPos() {
        return new BucketPosCommand(extension, pivot, wrist);
    }

}
