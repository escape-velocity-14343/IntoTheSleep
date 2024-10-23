package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp
public class WristTest extends LinearOpMode {
    public WristSubsystem wrist;

    @Override
    public void runOpMode(){

        waitForStart();
        wrist = new WristSubsystem(hardwareMap);

        while (!isStopRequested()){
            wrist.periodic();
            telemetry.addData("nothing", 0);
        }
    }
}
