package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="Fix", name = "reset to sample auton sub pos")

public class ResetEverything extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        pinpoint.reset();
        imu.resetYaw();
        pinpoint.setPosition(-65,40);
        extension.reset();
        cs.reset();
    }
}
