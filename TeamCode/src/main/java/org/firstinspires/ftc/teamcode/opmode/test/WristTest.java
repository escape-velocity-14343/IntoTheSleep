package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Config
@TeleOp
public class WristTest extends LinearOpMode {
    public WristSubsystem wrist;
    public static double pos = 0;

    @Override
    public void runOpMode(){

        waitForStart();
        wrist = new WristSubsystem(hardwareMap);


        while (!isStopRequested()){
            wrist.setWrist(pos);
        }
    }
}
