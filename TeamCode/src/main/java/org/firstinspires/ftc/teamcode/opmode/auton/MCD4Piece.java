package org.firstinspires.ftc.teamcode.opmode.auton;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

//@Config
@Disabled
//@Autonomous(name = "Macca's 4 Piece Nuggets")
public class MCD4Piece extends Robot {
    private DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d scorePos = new Pose2d(-59, 55, Rotation2d.fromDegrees(-45));
        initialize();

        pinpoint.reset();

        wrist.setWrist(1);
        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 38, new Rotation2d()));

        pinpoint.setPosition(-65, 39);

        cs.schedule(new SequentialCommandGroup(
                // preload sample
                //new StayAtPointCommand(new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)), gtpc),
                new GoToPointWithDefaultCommand(scorePos, gtpc).alongWith(
                        new BucketPosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),



                // first sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 46.5, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),

                // second sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 56, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                new WaitCommand(100),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),

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

                /*new GoToPointWithDefaultCommand(new Pose2d(-38, 50, Rotation2d.fromDegrees(60)), gtpc)
                        .alongWith(new IntakePosCommand(extension, pivot, wrist))
                        .withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, IntakeConstants.autoIntakeSpeed),
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
                                        heading -= 0.025;
                                        gtpc.setTarget(new Pose2d(gtpc.getTargetX(), gtpc.getTargetY(), Rotation2d.fromDegrees(heading)));
                                    }
                                }
                        )
                        //new DrivebasePowerCommand(mecanum, 0.0, 0.0, 0.1)
                ),*/

                new GoToPointWithDefaultCommand(new Pose2d(-44, 58, Rotation2d.fromDegrees(18)), gtpc).alongWith(
                        new IntakePosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos - 0.02, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece3Extension).withTimeout(1500),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                new BucketPosCommand(extension, pivot, wrist),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),

                // park
                new GoToPointWithDefaultCommand(new Pose2d(-12, 40, Rotation2d.fromDegrees(90)), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-12, 17, Rotation2d.fromDegrees(90)), gtpc).withTimeout(3000),
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

            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());

        }
        cs.reset();
    }
}
