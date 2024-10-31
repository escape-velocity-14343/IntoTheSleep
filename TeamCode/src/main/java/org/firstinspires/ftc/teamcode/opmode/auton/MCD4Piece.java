package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.StayAtPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous
public class MCD4Piece extends Robot {
    private DefaultGoToPointCommand gtpc;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d scorePos = new Pose2d(-56,56, Rotation2d.fromDegrees(-45));
        initialize();
        otos.setPosition(-65, 39);
        waitForStart();

        extension.reset();


        gtpc = new DefaultGoToPointCommand(mecanum, otos, new Pose2d(-65, 39, new Rotation2d()));
        cs.schedule(gtpc);

        cs.schedule(
                new SequentialCommandGroup(
                        new StayAtPointCommand(new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)), gtpc),
                        new BucketPosCommand(extension, pivot, wrist),
                        new StayAtPointCommand(new Pose2d(-56,56, Rotation2d.fromDegrees(-45)), gtpc),
                        new IntakeSpinCommand(intake,IntakeConstants.autoOuttakeSpeed),
                        new WaitCommand(1500),
                        new IntakeSpinCommand(intake, 0),


                        new StayAtPointCommand(new Pose2d(-50, 48, Rotation2d.fromDegrees(0)), gtpc),
                        new IntakePosCommand(extension, pivot, wrist),
                        new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                        new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                        new WaitCommand(500),
                        new RetractCommand(wrist, pivot, extension),
                        new IntakeSpinCommand(intake, 0),
                        new StayAtPointCommand(new Pose2d(-56,56, Rotation2d.fromDegrees(-45)), gtpc),
                        new BucketPosCommand(extension, pivot, wrist),
                        new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                        new WaitCommand(1500),
                        new IntakeSpinCommand(intake, 0),


                        new StayAtPointCommand(new Pose2d(-48, 55, Rotation2d.fromDegrees(0)), gtpc),
                        new IntakePosCommand(extension, pivot, wrist),
                        new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                        new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                        new RetractCommand(wrist, pivot, extension),
                        new IntakeSpinCommand(intake, 0),
                        new WaitCommand(500),
                        new StayAtPointCommand(new Pose2d(-56,56, Rotation2d.fromDegrees(-45)), gtpc),
                        new BucketPosCommand(extension, pivot, wrist),
                        new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                        new WaitCommand(1500),
                        new IntakeSpinCommand(intake, 0)
        ));

        while (!isStopRequested()){
            update();

            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading",otos.getPose().getRotation().getDegrees());

        }
        cs.reset();
    }
}
