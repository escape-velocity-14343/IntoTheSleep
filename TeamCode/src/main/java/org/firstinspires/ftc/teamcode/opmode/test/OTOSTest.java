package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Disabled
@TeleOp(group="1")
public class OTOSTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        pinpoint.reset();
        waitForStart();
        while (!isStopRequested()){
            update();
            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());
            telemetry.addData("imu heading", imu.getRobotYawPitchRollAngles().getYaw());
        }
    }
}
