package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
@Config
public class TeleOpTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        GamepadEx driverPad = new GamepadEx(gamepad1);
        GamepadEx operatorPad = new GamepadEx(gamepad2);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        if (true) {
            CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                    mecanum,
                    driverPad::getLeftY,
                    driverPad::getLeftX,
                    driverPad::getRightX,
                    () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
            ));
        } else {
            DefaultDriveCommand drive = new DefaultDriveCommand(mecanum, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX, () -> 0.0);
            CommandScheduler.getInstance().setDefaultCommand(mecanum, drive);
        }

//        CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
//                mecanum,
//                driverPad::getLeftY,
//                driverPad::getLeftX,
//                driverPad::getRightX,
//                () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
//        ));
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakePos());
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(subPos(),
                new IntakeSpinCommand(intake, 1),
                new InstantCommand(() -> extension.setManualControl(true), extension)));
        //driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new SequentialCommandGroup(new IntakeSpinCommand(intake, 0),
                        new InstantCommand(() -> extension.setManualControl(false), extension),
                        new RetractCommand(wrist, pivot, extension)));
        new Trigger(extension::getManualControl).toggleWhenActive(new RunCommand(() -> extension.setPower(gamepad2.right_trigger - gamepad2.left_trigger), extension));
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeSpinCommand(intake, -0.2));
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(bucketPos());
        new Trigger(() -> gamepad1.options).whileActiveOnce(new InstantCommand(() -> imu.resetYaw()));
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            timer.reset();
            update();
        }
    }
}
