package org.firstinspires.ftc.teamcode.opmode.auton.SubmersibleSample;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.alliance;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.group.Auton3Yellows;
import org.firstinspires.ftc.teamcode.commands.group.AutonSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


// 6+0 Pre IL
//@Autonomous(name = "6 Sample 0 Specimen Popeyes' 6 Piece")
public abstract class PI6Piece extends Robot {
    private DefaultGoToPointCommand gtpc;
    private CameraSubsystem cam;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        cam = new CameraSubsystem(hardwareMap, intake::getDSensorSupplier);

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);

        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        pinpoint.setPosition(-65, 41);


        cs.schedule(false, new SequentialCommandGroup(

                new GoToPointWithDefaultCommand(scorePos, gtpc).alongWith(
                        new BucketPosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                //new IntakeClawCommand(intake, IntakeConstants.closedPos),

                new Auton3Yellows(extension, pivot, wrist, intake, gtpc),

                // sub cycle 1
                new AutonSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, true),
                new AutonSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, false),

                // park
                new SequentialCommandGroup(new GoToPointWithDefaultCommand(new Pose2d(-6, 40, Rotation2d.fromDegrees(90)), gtpc, 20, 20)
                        .interruptOn(() -> pinpoint.getPose().getX() > -20), new GoToPointWithDefaultCommand(new Pose2d(-12, 17.5, Rotation2d.fromDegrees(90)), gtpc).withTimeout(500)

                )
                        .alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new PivotCommand(pivot, PivotConstants.parkDegrees)



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