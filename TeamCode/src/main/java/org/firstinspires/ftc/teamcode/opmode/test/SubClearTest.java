package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;

@Config
@TeleOp(group="1")
public class SubClearTest extends LinearOpMode {

    public static double position = 0;

    @Override
    public void runOpMode() {
        SubClearSubsystem subClear = new SubClearSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            subClear.setPosition(position);
        }
    }

}
