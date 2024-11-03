package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.IMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="1")
public class OTOSTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        otos.reset();
        waitForStart();
        while (!isStopRequested()){
            update();
            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading", otos.getPose().getRotation().getDegrees());
            telemetry.addData("imu heading", imu.getRobotYawPitchRollAngles().getYaw());
        }
    }
}
