package org.firstinspires.ftc.teamcode.opmode.auton.SubmersibleSample;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.Auton3Yellows;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


@Config
//@Autonomous(name = "6 Sample 0 Specimen Popeyes' 6 Piece")
public abstract class O7Piece extends Robot {
    private DefaultGoToPointCommand gtpc;
    private CameraSubsystem cam;

    public static Pose2d partnerPos = new Pose2d(-56, 0, Rotation2d.fromDegrees(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        cam = new CameraSubsystem(hardwareMap, intake::getDSensorSupplier);
        cam.waitForSetExposure(3000,5000);

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
                ),//.withTimeout(3000),
                new IntakeControlCommand(intake,IntakeConstants.openPos, -1),
                new WaitCommand(250),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                //new IntakeClawCommand(intake, IntakeConstants.closedPos),

                new GoToPointWithDefaultCommand(partnerPos, gtpc).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos).andThen(new IntakePosCommand(extension, pivot, wrist))
                ).withTimeout(3000),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension).interruptOn(intake::getDSensorSupplier),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                //RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos)
                new BucketPosCommand(extension, pivot, wrist).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                //new WaitCommand(100),

                new IntakeControlCommand(intake,IntakeConstants.openPos, -1),
                new WaitCommand(350),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),

                new Auton3Yellows(extension, pivot, wrist, intake, gtpc, cam, pinpoint),

                // sub cycle 1
                new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, true),
                new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, false),
                //new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, true, new Pose2d(-4, 24, Rotation2d.fromDegrees(-90))),
                //new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, false, new Pose2d(-4, 24, Rotation2d.fromDegrees(-90))),
                //new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, true, new Pose2d(2, 22, Rotation2d.fromDegrees(-90))),
                //new AutoSubCycle(extension, pivot, wrist, intake, cam, subClear, pinpoint, gtpc, false, new Pose2d(2, 22, Rotation2d.fromDegrees(-90))),

                // park
                new SequentialCommandGroup(new GoToPointWithDefaultCommand(new Pose2d(-8, 50, Rotation2d.fromDegrees(90)), gtpc, 20, 20)
                        .interruptOn(() -> pinpoint.getPose().getX() > -20), new GoToPointWithDefaultCommand(new Pose2d(-12, 17.5, Rotation2d.fromDegrees(90)), gtpc).withTimeout(500)

                )
                        .alongWith(
                                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
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