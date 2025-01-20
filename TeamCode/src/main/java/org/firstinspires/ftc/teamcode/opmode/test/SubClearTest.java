package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;

@Config
@TeleOp(group="Test")
public class SubClearTest extends LinearOpMode {

    public static double position = 0;
    public static double position2 = 0;

    @Override
    public void runOpMode() {
        SubClearSubsystem subClear = new SubClearSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            subClear.dualSetPosition(position, position2);
        }
        CommandScheduler.getInstance().reset();
    }

}
