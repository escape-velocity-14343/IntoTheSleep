package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
public class IntakeTest extends LinearOpMode {
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        waitForStart();

        while (!isStopRequested()){
            intakeSubsystem.periodic();
            telemetry.addData("nothing", 0);
        }
    }
}