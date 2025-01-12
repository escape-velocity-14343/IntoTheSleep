package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous (group = "Test")
@Config
public class VelocityCompensatedSquIDTuner extends Robot {

    public static double power = 0.1;

    public static double seconds = 0.5;

    @Override
    public void runOpMode() {
        initialize();

        pinpoint.reset();
        waitForStart();
        pinpoint.resetYaw();
        pinpoint.setPosition(0, 0);

        ElapsedTime timer = new ElapsedTime();
        // move mec
        while (timer.seconds() < seconds && opModeIsActive()) {
            mecanum.driveFieldCentric(power, 0, 0, 0);
            pinpoint.periodic();
        }

        // store post-movement
        mecanum.driveFieldCentric(0, 0, 0, 0);
        telemetry.addData("Velocity", pinpoint.getVelocity().getX());
        Pose2d initialPosition = pinpoint.getPose();

        while (pinpoint.getVelocity().getTranslation().getNorm() > 0.0001 && opModeIsActive()) {
            // wait :P
            pinpoint.periodic();
        }

        Pose2d finishedPosition = pinpoint.getPose();
        telemetry.addData("Final Distance", initialPosition.getTranslation().getDistance(finishedPosition.getTranslation()));
        telemetry.update();
    }

}
