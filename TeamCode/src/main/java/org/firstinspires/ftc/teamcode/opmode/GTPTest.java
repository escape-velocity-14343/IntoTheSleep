package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class GTPTest extends Robot {
    public static double targetx = 65;
    public static double targety = -42;
    public static double targetrot = 0;
    CommandScheduler cs = CommandScheduler.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        otos.setPosition(65, -42);
        //cs.schedule(new WristCommand(wrist, IntakeConstants.groundPos));
        //cs.schedule(new IntakeSpinCommand(intake, -1.0));
        //cs.schedule(new GoToPointCommand(mecanum, otos, new Pose2d(36, 40, new Rotation2d())));

        waitForStart();
        cs.schedule(new GoToPointCommand(mecanum, otos, new Pose2d(targetx, targety, new Rotation2d(targetrot))));
        while (!isStopRequested()){
            update();
            //cs.schedule(new GoToPointCommand(mecanum, otos, new Pose2d(48, 36, new Rotation2d())));


            telemetry.addData("x", otos.getPose().getX());
            telemetry.addData("y", otos.getPose().getY());
            telemetry.addData("heading",otos.getPose().getHeading());


        }
        cs.reset();
    }
}
