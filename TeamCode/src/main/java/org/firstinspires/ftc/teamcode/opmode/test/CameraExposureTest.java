package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExposureCycle;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.SampleAutoAlignAndExtend;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Camera Exposure Test", group = "Test")
public class CameraExposureTest extends Robot {
    CameraSubsystem cam;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = new CameraSubsystem(hardwareMap, () -> false);

        initialize();
        cam.waitForSetExposure(3000,5000);

        waitForStart();

        cs.schedule(
                new ExposureCycle(cam)
        );


        while (!isStopRequested()) {
            update();
        }
        cs.reset();
    }
}
