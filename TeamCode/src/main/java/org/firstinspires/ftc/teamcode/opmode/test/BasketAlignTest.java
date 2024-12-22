package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.BasketAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.BasketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/** @noinspection unused*/
@TeleOp
public class BasketAlignTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        pinpoint.resetYaw();
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem("frontRight", "frontLeft", "backRight", "backLeft", hardwareMap, pinpoint);
        BasketSensorSubsystem basketSensor = new BasketSensorSubsystem(hardwareMap);

        waitForStart();

        BasketAlignCommand basketCommand = new BasketAlignCommand(drive, basketSensor, pinpoint);

        CommandScheduler.getInstance().schedule(basketCommand);

        while (!isStopRequested()) {
            telemetry.addData("left sensor", basketSensor.getSensorLeft());
            telemetry.addData("right sensor", basketSensor.getSensorRight());
            telemetry.addData("error", basketCommand.error);

            telemetry.update();
            CommandScheduler.getInstance().run();
        }
    }
}
