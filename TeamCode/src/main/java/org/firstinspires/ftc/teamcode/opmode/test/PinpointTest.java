package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="Test")
public class PinpointTest extends Robot {


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        waitForStart();

        while (opModeIsActive()) {
            update();
            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());
            telemetry.addData("imu heading", imu.getRobotYawPitchRollAngles().getYaw());
            int[] enc = pinpoint.getEncoderCounts();
            telemetry.addData("x enc",  enc[0]);
            telemetry.addData("y enc", enc[1]);
        }

    }
}
