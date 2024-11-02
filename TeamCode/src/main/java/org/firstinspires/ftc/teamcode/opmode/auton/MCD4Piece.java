package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous
public class MCD4Piece extends Robot {
    private DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d scorePos = new Pose2d(-56, 54, Rotation2d.fromDegrees(-45));
        initialize();
        otos.setPosition(-65, 39);
        wrist.setWrist(1);
        waitForStart();

        extension.reset();


        gtpc = new DefaultGoToPointCommand(mecanum, otos, new Pose2d(-65, 39, new Rotation2d()));

        cs.schedule(new SequentialCommandGroup(
                // preload sample
                //new StayAtPointCommand(new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)), gtpc),
                new GoToPointWithDefaultCommand(scorePos, gtpc).alongWith(
                        new BucketPosCommand(extension, pivot, wrist)
                ),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1000),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),


                // first sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 48, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.scoringPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                new IntakeSpinCommand(intake, 0),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1000),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),

                // second sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 58, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.scoringPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ).withTimeout(4000),
                new IntakeSpinCommand(intake, 0),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1000),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),

                // front wall sample
                /*new StayAtPointCommand(new Pose2d(-44, 57.5, Rotation2d.fromDegrees(28)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                new AutonExtendCommand(extension, SlideConstants.autonPiece3Extension).withTimeout(1500),
                new Retract Command(wrist, pivot, extension).alongWith(
                        new StayAtPointCommand(scorePos, gtpc)
                ),
                new IntakeSpinCommand(intake, 0),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1000),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.groundPos),*/

                // third sample
                //new GoToPointWithDefaultCommand(new Pose2d(scorePos.getX(), 50, Rotation2d.fromDegrees(90)), gtpc).withTimeout(500),
                new GoToPointWithDefaultCommand(new Pose2d(-38, 50, Rotation2d.fromDegrees(60)), gtpc)
                        .alongWith(new IntakePosCommand(extension, pivot, wrist))
                        .withTimeout(3000),
                new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                new ParallelDeadlineGroup(
                        new WaitCommand(1500).alongWith(
                                new InstantCommand(() -> {
                                    extension.setManualControl(true);
                                    extension.setPower(SlideConstants.autonPiecee3ExtensionPower);
                                })),
                        new SequentialCommandGroup(
                                new WaitCommand(750),
                                new CommandBase() {
                                    private double heading = gtpc.getTargetHeading();

                                    @Override
                                    public void execute() {
                                        heading -= 0.1;
                                        gtpc.setTarget(new Pose2d(gtpc.getTargetX(), gtpc.getTargetY(), Rotation2d.fromDegrees(heading)));
                                    }
                                }
                        )
                        //new DrivebasePowerCommand(mecanum, 0.0, 0.0, 0.1)
                ),
                new IntakeSpinCommand(intake, 0),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.scoringPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                new IntakeSpinCommand(intake, 0),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1000),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.groundPos),

                // park
                new GoToPointWithDefaultCommand(new Pose2d(-12, 40, Rotation2d.fromDegrees(90)), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-12, 22, Rotation2d.fromDegrees(90)), gtpc).withTimeout(3000),
                new PivotCommand(pivot, PivotConstants.parkDegrees)





                /*new StayAtPointCommand(new Pose2d(-48, 57, Rotation2d.fromDegrees(0)), gtpc),
                new IntakePosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                new RetractCommand(wrist, pivot, extension),
                new IntakeSpinCommand(intake, 0),
                new WaitCommand(100),
                new StayAtPointCommand(scorePos, gtpc),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                new WaitCommand(1500),
                new IntakeSpinCommand(intake, 0)*/
        ));

        cs.schedule(gtpc);
        while (!isStopRequested()) {
            update();

            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading", otos.getPose().getRotation().getDegrees());

        }
        cs.reset();
    }
}
