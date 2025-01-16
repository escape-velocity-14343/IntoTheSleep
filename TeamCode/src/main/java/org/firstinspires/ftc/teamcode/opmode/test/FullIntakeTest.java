package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Config
@TeleOp(group="Test")
public class FullIntakeTest extends LinearOpMode {
    public WristSubsystem wrist;
    public IntakeSubsystem intakeSubsystem;
    public static double pos = 0;
    public static double speed = 0;
    public static double claw = 0;

    @Override
    public void runOpMode(){

        waitForStart();
        wrist = new WristSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(wrist, intakeSubsystem);


        while (!isStopRequested()){
            wrist.setWrist(pos);
            intakeSubsystem.setIntakeSpeed(speed);
            intakeSubsystem.setClawer(claw);

        }
        CommandScheduler.getInstance().reset();
    }
}
