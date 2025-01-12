package org.firstinspires.ftc.teamcode.opmode.test;

import androidx.core.location.GnssStatusCompat;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="1")
@Config
public class VelocityCompensatedSquIDTester extends Robot {

    public static double x = 0;
    public static double y = 0;
    public static double rot = 0;
    private DefaultGoToPointCommand gtpc;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        pinpoint.reset();

        waitForStart();
        pinpoint.setPosition(0, 0);

        //cs.schedule(
        //        new SequentialCommandGroup(
        //                new ParallelCommandGroup(
        //                        new BucketPosCommand(extension, pivot, wrist), new GoToPointCommand(mecanum, otos, new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)))),
        //                new GoToPointCommand(mecanum, otos, new Pose2d(-54, 54, Rotation2d.fromDegrees(-45))),
        //                new IntakeSpinCommand(intake,-0.4),
        //                new WaitCommand(1000),
        //                new GoToPointCommand(mecanum, otos, new Pose2d(-54, 48, new Rotation2d(0))).alongWith(new WristCommand(wrist, IntakeConstants.groundPos)),
        //                new IntakePosCommand(extension, pivot, wrist)
        //        ));
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        ElapsedTime timer = new ElapsedTime();
        cs.schedule(false, new SequentialCommandGroup(
                new GoToPointWithDefaultCommand(new Pose2d(x, y, Rotation2d.fromDegrees(rot)), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(0, 0, new Rotation2d()), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(x, y, Rotation2d.fromDegrees(rot)), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(0, 0, new Rotation2d()), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(x, y, Rotation2d.fromDegrees(rot)), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(0, 0, new Rotation2d()), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(x, y, Rotation2d.fromDegrees(rot)), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(0, 0, new Rotation2d()), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(x, y, Rotation2d.fromDegrees(rot)), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                }),
                new WaitCommand(2000),
                new InstantCommand(timer::reset),
                new GoToPointWithDefaultCommand(new Pose2d(0, 0, new Rotation2d()), gtpc).whenFinished(() -> {
                    telemetry.addData("Seconds", timer.seconds());
                    telemetry.update();
                    timer.reset();
                })
                )
        );

        cs.schedule(gtpc);

        while (!isStopRequested()){
            update();
        }
        cs.reset();
    }
}
