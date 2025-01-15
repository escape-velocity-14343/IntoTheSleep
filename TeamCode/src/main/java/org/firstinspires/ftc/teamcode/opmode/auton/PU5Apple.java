package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSpecimenScoreCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

// 5 Specimen pre-semis
@Config
@Autonomous(name="5 Specimen Puyallup 5 Apple")
public class PU5Apple extends Robot {


    private DefaultGoToPointCommand gtpc;
    public static Pose2d intakePos = new Pose2d(-63, -40, new Rotation2d());
    public static double intakeStallVelocity = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        pinpoint.reset();
        imu.resetYaw();
        extension.reset();

        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);
        //pivot.setTarget(PivotConstants.specimenTopBarAngle);

        sleep(1000);
        pinpoint.setPosition(-65, -8);
        /*while (opModeInInit() && !isStopRequested() && !isStarted()) {
            pivot.periodic();
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }*/

        waitForStart();


        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, -8, new Rotation2d()));



        cs.schedule(new SequentialCommandGroup(
                // score initial
                new SpecimenHookCommand(pivot, extension, wrist, intake),//.withTimeout(1000),
                new GoToPointWithDefaultCommand(new Pose2d(-30, -12, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -33.5),//.withTimeout(1500),
                new WristCommand(wrist, IntakeConstants.foldedPos).alongWith(new IntakeControlCommand(intake, IntakeConstants.openPos,0)),
                // push first
                new GoToPointWithDefaultCommand(new Pose2d(-50, -48, new Rotation2d()), gtpc, 5, 10).alongWith(
                        new RetractCommand(wrist, pivot, extension).andThen(
                                new ExtendCommand(extension, 2).alongWith(
                                    new WristCommand(wrist, IntakeConstants.groundPos),
                                    new IntakeControlCommand(intake, IntakeConstants.openPos,1)
                                )
                        )//.withTimeout(2000)
                ),
                new GoToPointWithDefaultCommand(new Pose2d(-34, -48, new Rotation2d()), gtpc),
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
                new GoToPointWithDefaultCommand(new Pose2d(-34, -60, new Rotation2d()), gtpc),
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
                new GoToPointWithDefaultCommand(new Pose2d(-18, -68, new Rotation2d()), gtpc).andThen(
                        new GoToPointWithDefaultCommand(new Pose2d(-58, -66, new Rotation2d()), gtpc, 5, 5).interruptOn(() -> pinpoint.getVelocity().getTranslation().getNorm() < PU5Apple.intakeStallVelocity)
                        )
                ),

                // score first
                new GoToPointWithDefaultCommand(new Pose2d(intakePos.getX(), -65, new Rotation2d()), gtpc).alongWith(new IntakeControlCommand(intake, IntakeConstants.openPos, 1)).interruptOn(() -> pinpoint.getVelocity().getTranslation().getNorm() < PU5Apple.intakeStallVelocity),
                new WaitCommand(300).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos)),
                //new GoToPointWithDefaultCommand(new Pose2d(-50, -8, new Rotation2d()), gtpc, 5, 10)).alongWith(
                //        new SpecimenHookCommand(pivot, extension, wrist, intake)
                //),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, -14, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > -12)).withTimeout(1500),
                new GoToPointWithDefaultCommand(new Pose2d(-30, -6, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -33).withTimeout(1500),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),

                // score second
                new AutoSpecimenScoreCommand(pivot, extension, wrist, intake, gtpc, pinpoint, -4),
                // score third
                new AutoSpecimenScoreCommand(pivot, extension, wrist, intake, gtpc, pinpoint, 0),
                // score fourth
                new AutoSpecimenScoreCommand(pivot, extension, wrist, intake, gtpc, pinpoint, 4),
                
                // park
                new GoToPointWithDefaultCommand(new Pose2d(-63, -40, Rotation2d.fromDegrees(45)), gtpc)
             )
        );

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
