package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="Fix")
public class WristOffset extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        Gamepad g1clone = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            g1clone.copy(gamepad1);
            if (g1clone.dpad_up && !lastDpadUp) {
                IntakeConstants.wristOffset += 0.01;
            }
            if (g1clone.dpad_down && !lastDpadDown) {
                IntakeConstants.wristOffset -= 0.01;
            }
            lastDpadUp = g1clone.dpad_up;
            lastDpadDown = g1clone.dpad_down;
            telemetry.addData("Wrist Offset", IntakeConstants.wristOffset);
            telemetry.update();
        }
    }
}
