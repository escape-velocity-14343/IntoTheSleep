package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class OTOSTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested()){
            update();
            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading",otos.getPose().getHeading());
        }
    }
}
