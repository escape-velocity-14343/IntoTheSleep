package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
public class TeleOpTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        GamepadEx driverPad = new GamepadEx(gamepad1);
        DefaultDriveCommand drive = new DefaultDriveCommand(mecanum, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(mecanum,drive);
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakePos());
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(intakeEat(), new IntakeSpinCommand(intake, 1)));
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(outtake());
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            timer.reset();
            update();
        }
    }
}
