package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name = "Chick-Fil-A Combo Meal (4 Cows, No Pickles Please))")
public class CFA4Piece extends Robot {
    private DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d scorePos = new Pose2d(-59, 55, Rotation2d.fromDegrees(-45));
        initialize();

        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);
        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));

        pinpoint.setPosition(-65, 40);

        cs.schedule(new SequentialCommandGroup(

                new GoToPointWithDefaultCommand(scorePos, gtpc).alongWith(
                        new BucketPosCommand(extension, pivot, wrist)
                ).withTimeout(3000),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                new IntakeClawCommand(intake, IntakeConstants.closedPos),

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
