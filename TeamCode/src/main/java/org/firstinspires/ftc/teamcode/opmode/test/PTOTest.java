package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(group="1")
public class PTOTest extends Robot {
    public static AscentSubsytem.PTOMode engagePto = AscentSubsytem.PTOMode.ENGAGED;

    @Override
    public void runOpMode() {
        initialize();

        while (opModeInInit()) {
            pto.setPto(engagePto);
        }

        waitForStart();

        cs.schedule(new PivotCommand(pivot, PivotConstants.bottomLimit));

        while (opModeIsActive()) {
            pto.setPto(engagePto);
            if (Math.abs(gamepad1.right_stick_y) > 0.01) {
                pto.move(-gamepad1.right_stick_y);
            } else {
                pto.debugMove(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            }

            update();
        }
        cs.reset();
    }

}
