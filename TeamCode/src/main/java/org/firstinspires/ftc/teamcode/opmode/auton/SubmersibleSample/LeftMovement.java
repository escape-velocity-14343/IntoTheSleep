package org.firstinspires.ftc.teamcode.opmode.auton.SubmersibleSample;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.alliance;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


// 6+0 Pre IL
@Disabled
@Autonomous(name = "LEFT movement")
public class LeftMovement extends Robot {
    CameraSubsystem cam;
    private DefaultGoToPointCommand gtpc;

    private Pose2d subIntakePos = new Pose2d(6, 22, Rotation2d.fromDegrees(-90));
    //private Pose2d subIntakePos = new Pose2d(-13, 22, Rotation2d.fromDegrees(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        alliance = AutoConstants.Alliance.BLUE;

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);
        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        pinpoint.setPosition(-65, 40);
        while (!cam.setExposure());

        cs.schedule(false, new SequentialCommandGroup(
                // sub cycle 1
                new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, true, subIntakePos),
                new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, false, subIntakePos)
        ));

        cs.schedule(gtpc);
        while (!isStopRequested()) {
            update();

            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());

        }
        end();
    }
}