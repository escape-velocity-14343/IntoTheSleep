package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.SampleAutoAlignAndExtend;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Autonomous Align Testing", group = "Test")
public class AutoAlignTest extends Robot {
    CameraSubsystem cam;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = new CameraSubsystem(hardwareMap, () -> false);

        initialize();
        cam.waitForSetExposure(3000,5000);

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.groundPos);
        intake.setClawer(IntakeConstants.singleIntakePos);
        waitForStart();
        intake.setIntakeSpeed(1);

        imu.resetYaw();
        extension.reset();

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(0, 0, new Rotation2d()));

        pinpoint.setPosition(0, 0);


        cs.schedule(new SequentialCommandGroup(
                new SampleAutoAlignAndExtend(cam, gtpc, pinpoint, extension, intake)
        ));

        cs.schedule(gtpc);
        while (!isStopRequested()) {
            update();

            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());

        }
        cs.reset();
    }
}
