package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.BasketAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.BasketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/** @noinspection unused*/
@TeleOp(group="z")
public class UltrasonicTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        BasketSensorSubsystem basketSensor = new BasketSensorSubsystem(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {
            telemetry.addData("left sensor", basketSensor.getSensorLeft());
            telemetry.addData("right sensor", basketSensor.getSensorRight());
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
    }
}
