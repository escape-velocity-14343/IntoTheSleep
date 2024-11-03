package org.firstinspires.ftc.teamcode.opmode.teleop;

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

import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(group="0", name="TeleOpp")
@Config
public class TeleOpps extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {

        // TODO (post lm1): refactor this into fsm states
        AtomicBoolean inHang = new AtomicBoolean(false);

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
                    () -> otos.getPose().getRotation().getDegrees()
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
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist, pivot, extension));
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> inHang.set(false)),
                subPos(),
                new IntakeSpinCommand(intake, 1).interruptOn(()->operatorPad.isDown(GamepadKeys.Button.LEFT_BUMPER)),
                new InstantCommand(() -> extension.setManualControl(true), extension))
        );
        //driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new SequentialCommandGroup(
                        new IntakeSpinCommand(intake, 0),
                        new InstantCommand(() -> extension.setManualControl(false), extension),
                        new IntakeRetractCommand(wrist, pivot, extension))
                );
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeSpinCommand(intake, -0.15));
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(bucketPos().andThen(new InstantCommand(() -> inHang.set(false))));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new RetractCommand(wrist, pivot, extension)
                .andThen(new PivotCommand(pivot, PivotConstants.topLimit),
                        new InstantCommand(() -> inHang.set(true))));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ConditionalCommand(
            new PivotCommand(pivot, PivotConstants.hangDegrees), new InstantCommand(), inHang::get));


        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(extension::reset));
        operatorPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> extension.setManualControl(true)));
        operatorPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> extension.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> pivot.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> pivot.setManualControl(true)));

        operatorPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeSpinCommand(intake, 1))
                .whenReleased(new IntakeSpinCommand(intake, 0));

        operatorPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeSpinCommand(intake, -1))
                .whenReleased(new IntakeSpinCommand(intake, 0));

        new Trigger(extension::getManualControl).toggleWhenActive(new RunCommand(() -> extension.setPower(gamepad2.right_trigger + gamepad1.right_trigger - gamepad2.left_trigger - gamepad1.left_trigger), extension));
                /*new RunCommand(() -> extension.setPower(- gamepad2.left_trigger - gamepad1.left_trigger), extension),
                () -> extension.getCurrentPosition() / SlideConstants.ticksPerInch < SlideConstants.submersibleIntakeMaxExtension))*/

        new Trigger(pivot::getManualControl).toggleWhenActive(new RunCommand(() -> pivot.setPower(gamepad2.right_stick_x), pivot));


        new Trigger(() -> gamepad1.options).whileActiveOnce(new InstantCommand(() -> otos.resetYaw()));
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            timer.reset();
            update();
        }
        CommandScheduler.getInstance().reset();
    }
}
