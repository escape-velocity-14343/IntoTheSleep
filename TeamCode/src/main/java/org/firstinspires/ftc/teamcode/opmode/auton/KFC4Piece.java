package org.firstinspires.ftc.teamcode.opmode.auton;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
//@Config
//@Autonomous
public class KFC4Piece extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        pinpoint.setPosition(-65, 39);
        waitForStart();
        extension.reset();

        cs.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new BucketPosCommand(extension, pivot, wrist), new GoToPointCommand(mecanum, pinpoint, new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)))),
                        new GoToPointCommand(mecanum, pinpoint, new Pose2d(-53, 53, Rotation2d.fromDegrees(-45))),
                        new IntakeSpinCommand(intake,IntakeConstants.autoOuttakeSpeed),
                        new WaitCommand(1500),
                        new IntakeSpinCommand(intake, 0),
                        new GoToPointCommand(mecanum, pinpoint, new Pose2d(-54, 48, new Rotation2d(0))),
                        new IntakePosCommand(extension, pivot, wrist).alongWith(new GoToPointCommand(mecanum, pinpoint, new Pose2d(-54, 48, new Rotation2d(0)), 0)),
                        new IntakeSpinCommand(intake, IntakeConstants.autoIntakeSpeed),
                        new WaitCommand(500),
                        new ExtendCommand(extension, SlideConstants.autonPiece1Extension).alongWith((new GoToPointCommand(mecanum, pinpoint, new Pose2d(-54, 48, new Rotation2d(0)), 0))),
                        new RetractCommand(wrist, pivot, extension),
                        new IntakeSpinCommand(intake, 0),
                        new ParallelCommandGroup(
                                new GoToPointCommand(mecanum, pinpoint, new Pose2d(-53, 53, Rotation2d.fromDegrees(-45))), new BucketPosCommand(extension, pivot, wrist)),
                        new IntakeSpinCommand(intake, IntakeConstants.autoOuttakeSpeed),
                        new WaitCommand(1500),
                        new IntakeSpinCommand(intake, 0)
        ));

        while (!isStopRequested()){
            update();

            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getHeading());

        }
        cs.reset();
    }
}
