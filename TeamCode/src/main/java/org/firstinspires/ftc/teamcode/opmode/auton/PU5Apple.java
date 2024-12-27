package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.Auton3Yellows;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

// 5 Specimen pre-semis
@Autonomous(name="5 Specimen Puyallup 5 Apple")
public class PU5Apple extends Robot {


    private DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);
        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, -8, new Rotation2d()));

        pinpoint.setPosition(-65, -8);

        cs.schedule(new SequentialCommandGroup(
                // score initial
                new SpecimenHookCommand(pivot, extension, wrist, intake),
                new GoToPointWithDefaultCommand(new Pose2d(-37, -12, new Rotation2d()), gtpc),

                // push first
                new GoToPointWithDefaultCommand(new Pose2d(-45, -34, new Rotation2d()), gtpc, 5, 5).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-18, -34, new Rotation2d()), gtpc, 5, 5),
                new GoToPointWithDefaultCommand(new Pose2d(-18, -48, new Rotation2d()), gtpc),
                new GoToPointWithDefaultCommand(new Pose2d(-56, -48, new Rotation2d()), gtpc, 5, 5),

                // push second
                new GoToPointWithDefaultCommand(new Pose2d(-18, -48, new Rotation2d()), gtpc),
                new GoToPointWithDefaultCommand(new Pose2d(-18, -57, new Rotation2d()), gtpc),
                new GoToPointWithDefaultCommand(new Pose2d(-56, -57, new Rotation2d()), gtpc, 5, 5),

                // push third
                new GoToPointWithDefaultCommand(new Pose2d(-18, -57, new Rotation2d()), gtpc),
                new SpecimenRaiseCommand(pivot, extension, wrist)
                        .alongWith(
                new GoToPointWithDefaultCommand(new Pose2d(-18, -65, new Rotation2d()), gtpc)
                        .andThen(
                        new GoToPointWithDefaultCommand(new Pose2d(-56, -65, new Rotation2d()), gtpc, 5, 5)
                        )
                ),

                // score first
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, new Rotation2d()), gtpc).alongWith(new IntakeControlCommand(intake, IntakeConstants.openPos, 1)),
                new WaitCommand(200).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new GoToPointWithDefaultCommand(new Pose2d(-45, -8, new Rotation2d()), gtpc).alongWith(
                        new SpecimenHookCommand(pivot, extension, wrist, intake)
                ),

                // score second
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, new Rotation2d()), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                                    new IntakeControlCommand(intake, IntakeConstants.openPos, 1)
                                )
                        )
                ),
                new WaitCommand(200).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new GoToPointWithDefaultCommand(new Pose2d(-45, -4, new Rotation2d()), gtpc).alongWith(
                        new SpecimenHookCommand(pivot, extension, wrist, intake)
                ),

                // score third
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, new Rotation2d()), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                                        new IntakeControlCommand(intake, IntakeConstants.openPos, 1)
                                )
                        )
                ),
                new WaitCommand(200).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new GoToPointWithDefaultCommand(new Pose2d(-45, 0, new Rotation2d()), gtpc).alongWith(
                        new SpecimenHookCommand(pivot, extension, wrist, intake)
                ),

                // score fourth
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, new Rotation2d()), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                                        new IntakeControlCommand(intake, IntakeConstants.openPos, 1)
                                )
                        )
                ),
                new WaitCommand(200).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new GoToPointWithDefaultCommand(new Pose2d(-45, 4, new Rotation2d()), gtpc).alongWith(
                        new SpecimenHookCommand(pivot, extension, wrist, intake)
                ),
                
                // park
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, new Rotation2d()), gtpc).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                )


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
