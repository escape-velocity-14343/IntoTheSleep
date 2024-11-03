package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="1")
@Config
public class GTPointTest extends Robot {
    public static double x = -65;
    public static double y = 39;
    public static double rot = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        otos.setPosition(-65, 39);
        waitForStart();

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

        cs.schedule(new GoToPointCommand(mecanum, otos, new Pose2d(x, y, Rotation2d.fromDegrees(rot))));

        while (!isStopRequested()){
            update();

            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading",otos.getPose().getHeading());

        }
        cs.reset();
    }
}
