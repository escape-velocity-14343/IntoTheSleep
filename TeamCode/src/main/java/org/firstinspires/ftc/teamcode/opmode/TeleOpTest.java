package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
public class TeleOpTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        GamepadEx driverPad = new GamepadEx(gamepad1);
        DefaultDrive drive = new DefaultDrive(mecanum, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(mecanum,drive);
        while (!isStopRequested()) {
            update();
            telemetry.addData("motorpos", extension.getCurrentPosition());
            extension.setPower(driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driverPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        }
    }
}
