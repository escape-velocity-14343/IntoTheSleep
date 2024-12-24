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
        //pinpoint.resetYaw();
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem("frontRight", "frontLeft", "backRight", "backLeft", hardwareMap, pinpoint);
        BasketSensorSubsystem basketSensor = new BasketSensorSubsystem(hardwareMap);

        waitForStart();

        BasketAlignCommand basketCommand = new BasketAlignCommand(drive, basketSensor, pinpoint)
                .withXySupplier(() -> gamepad1.touchpad_finger_1_y * 2.5, () -> gamepad1.touchpad_finger_1_x * 4);

        CommandScheduler.getInstance().schedule(basketCommand);

        while (!isStopRequested()) {
            telemetry.addData("left sensor", basketSensor.getSensorLeft());
            telemetry.addData("right sensor", basketSensor.getSensorRight());
            telemetry.addData("pinpoint x", pinpoint.getPose().getX());
            telemetry.addData("pinpoint y", pinpoint.getPose().getY());
            telemetry.addData("error", basketCommand.error);
            telemetry.addData("gp1 touchpad x", gamepad1.touchpad_finger_1_x);
            telemetry.addData("gp1 touchpad y", gamepad1.touchpad_finger_1_y);

            telemetry.update();
            CommandScheduler.getInstance().run();
        }
    }
}
