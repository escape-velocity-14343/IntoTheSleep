package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
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
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

// 5 Specimen pre-semis
@Autonomous(name="5 Specimen Puyallup 5 Apple")
public class PU5Apple extends Robot {


    private DefaultGoToPointCommand gtpc;
    Pose2d intakePos = new Pose2d(-62, -40, new Rotation2d());

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        pinpoint.reset();
        imu.resetYaw();
        extension.reset();

        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);
        pivot.setTarget(PivotConstants.specimenTopBarAngle);

        sleep(1000);
        pinpoint.setPosition(-65, -8);
        while (opModeInInit() && !isStopRequested() && !isStarted()) {
            pivot.periodic();
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }

        waitForStart();


        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, pinpoint.getPose());



        cs.schedule(new SequentialCommandGroup(
                // score initial
                new SpecimenHookCommand(pivot, extension, wrist, intake),
                new GoToPointWithDefaultCommand(new Pose2d(-30, -12, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -34),

                // push first
                new GoToPointWithDefaultCommand(new Pose2d(-50, -48, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new ExtendCommand(extension, 2).alongWith(
                                    new WristCommand(wrist, IntakeConstants.groundPos),
                                    new IntakeControlCommand(intake, IntakeConstants.openPos,1)
                                )
                        )
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-36, -48, new Rotation2d()), gtpc),
                new IntakeControlCommand(intake, IntakeConstants.closedPos,0),
                new GoToPointWithDefaultCommand(new Pose2d(-56, -48, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new SpecimenRaiseCommand(pivot, extension, wrist)
                ),
                new IntakeControlCommand(intake, 0.66, -1),

                // push second
                new GoToPointWithDefaultCommand(new Pose2d(-50, -60, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new ExtendCommand(extension, 2).alongWith(
                                        new WristCommand(wrist, IntakeConstants.groundPos),
                                        new IntakeControlCommand(intake, IntakeConstants.openPos,1)
                                )
                        )
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-36, -60, new Rotation2d()), gtpc),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),
                new GoToPointWithDefaultCommand(new Pose2d(-56, -60, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new SpecimenRaiseCommand(pivot, extension, wrist)
                ),
                new IntakeControlCommand(intake, 0.66, -1),


                // push third
                new GoToPointWithDefaultCommand(new Pose2d(-18, -57, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new RetractCommand(wrist, pivot, extension)
                ),
                new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                new GoToPointWithDefaultCommand(new Pose2d(-18, -65, new Rotation2d()), gtpc).andThen(
                        new GoToPointWithDefaultCommand(new Pose2d(-56, -65, new Rotation2d()), gtpc, 5, 5)
                        )
                ),

                // score first
                new GoToPointWithDefaultCommand(new Pose2d(-62.5, -65, new Rotation2d()), gtpc).alongWith(new IntakeControlCommand(intake, IntakeConstants.openPos, 1)),
                new WaitCommand(300).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                //new GoToPointWithDefaultCommand(new Pose2d(-50, -8, new Rotation2d()), gtpc, 5, 10)).alongWith(
                //        new SpecimenHookCommand(pivot, extension, wrist, intake)
                //),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, -14, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > -12)),
                new GoToPointWithDefaultCommand(new Pose2d(-30, -6, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -34),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),

                // score second
                new GoToPointWithDefaultCommand(new Pose2d(-58, -40, new Rotation2d()), gtpc).alongWith(
                        new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                            new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
                        )
                ),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new GoToPointWithDefaultCommand(intakePos, gtpc),
                new WaitCommand(300).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, 0, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > -8)),
                new GoToPointWithDefaultCommand(new Pose2d(-30, -4, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -34),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),

                // score third
                new GoToPointWithDefaultCommand(new Pose2d(-58, -40, new Rotation2d()), gtpc).alongWith(
                        new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
                        )
                ),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new GoToPointWithDefaultCommand(intakePos, gtpc),
                new WaitCommand(300).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, 4, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > 0)),
                new GoToPointWithDefaultCommand(new Pose2d(-30, 0, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -34),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                // score fourth
                new GoToPointWithDefaultCommand(new Pose2d(-58, -40, new Rotation2d()), gtpc).alongWith(
                        new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
                        )
                ),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new GoToPointWithDefaultCommand(intakePos, gtpc),
                new WaitCommand(300).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, 8, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > 4)),
                new GoToPointWithDefaultCommand(new Pose2d(-30, 4, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -34),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                
                // park
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, Rotation2d.fromDegrees(45)), gtpc)
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
