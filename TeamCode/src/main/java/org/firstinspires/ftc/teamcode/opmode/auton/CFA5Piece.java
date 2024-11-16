package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SlowerAutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.Auton3Yellows;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SampleAutoAlign;
import org.firstinspires.ftc.teamcode.commands.group.SubClearCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Chick-fil-a Combo Meal (4 Cows + 1 Pickle)")
public class CFA5Piece extends Robot {
    private DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d scorePos = new Pose2d(-59, 55, Rotation2d.fromDegrees(-45));
        CameraSubsystem cam = new CameraSubsystem(hardwareMap);
        cs.registerSubsystem(cam);

        initialize();

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.singleIntakePos+0.015);
        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 8, new Rotation2d()));

        pinpoint.setPosition(-65, 8);

        cs.schedule(new SequentialCommandGroup(
                new IntakeControlCommand(intake, 1, 0.8),
                new GoToPointWithDefaultCommand(new Pose2d(-32,5, new Rotation2d()),gtpc).alongWith(new SpecimenRaiseCommand(pivot, extension, wrist)),

                new SpecimenHookCommand(pivot, extension, wrist,intake),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                new GoToPointWithDefaultCommand(new Pose2d(-38,5,new Rotation2d(0)),gtpc).alongWith(new RetractCommand(wrist, pivot, extension)),

                new GoToPointWithDefaultCommand(new Pose2d(-32,5, new Rotation2d()),gtpc),
                new SubClearCommand(extension, pivot, wrist),
                new SubClearCommand(extension, pivot, wrist),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.openPos+0.025, 1),

                new SampleAutoAlign(cam, gtpc, pinpoint).deadlineWith(
                new SlowerAutonExtendCommand(extension, 16)).withTimeout(2000),

                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0.5),
                //new WaitCommand(1000),
                new IntakeRetractCommand(wrist, pivot, extension).alongWith(new GoToPointWithDefaultCommand(new Pose2d(-42, 5, new Rotation2d()),gtpc,8,20)),
                new PivotCommand(pivot, PivotConstants.topLimit-1).alongWith(
                new GoToPointWithDefaultCommand(scorePos, gtpc)),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),


                new Auton3Yellows(extension, pivot, wrist, intake, gtpc),
                // park
                new GoToPointWithDefaultCommand(new Pose2d(-12, 40, Rotation2d.fromDegrees(90)), gtpc, 20, 20).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-12, 17.5, Rotation2d.fromDegrees(90)), gtpc).withTimeout(500),
                new PivotCommand(pivot, PivotConstants.parkDegrees)

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
