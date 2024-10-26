package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
public class TeleOpTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        GamepadEx driverPad = new GamepadEx(gamepad1);
        GamepadEx operatorPad = new GamepadEx(gamepad2);
        DefaultDriveCommand drive = new DefaultDriveCommand(mecanum, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(mecanum,drive);

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakePos());
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(subPos(), new IntakeSpinCommand(intake, 1)));
        //driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new SequentialCommandGroup(new IntakeSpinCommand(intake, 0), new RetractCommand(wrist, pivot, extension)));
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeSpinCommand(intake, -0.5));
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(bucketPos());
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            timer.reset();
            update();
        }
    }
}
