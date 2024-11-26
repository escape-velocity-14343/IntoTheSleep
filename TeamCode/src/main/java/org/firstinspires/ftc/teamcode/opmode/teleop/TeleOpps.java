package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractReversedCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReversedCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group = "0", name = "TeleOpp")
@Config
public class TeleOpps extends Robot {

    public static double manualMultiplier = 0.5;
    public static double robotMovementMultiplier = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {


        initialize();
        GamepadEx driverPad = new GamepadEx(gamepad1);
        GamepadEx operatorPad = new GamepadEx(gamepad2);


        if (true) {
            CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                    mecanum,
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    //() -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                    () -> pinpoint.getPose().getRotation().getDegrees()
            ));
        } else {
            DefaultDriveCommand drive = new DefaultDriveCommand(mecanum,
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftY() / driverPad.getLeftX() < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftX() / driverPad.getLeftY() < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> 0.0);
            CommandScheduler.getInstance().setDefaultCommand(mecanum, drive);
        }

        //driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist, pivot, extension));

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new DefaultGoToPointCommand(mecanum, pinpoint, scorePos).interruptOn(
                        () -> (Math.abs(driverPad.getLeftX()) + Math.abs(driverPad.getLeftY()) + Math.abs(driverPad.getRightX())) > 0.05).alongWith(
                        new IntakeRetractCommand(wrist, pivot, extension).andThen(
                                //new WaitUntilCommand(() -> pinpoint.getPose().getX() < -24 && pinpoint.getPose().getY() > 24),
                                new BucketPosCommand(extension, pivot, wrist)
                        )
                )
        );

        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(subPos())
                .whenReleased(new SequentialCommandGroup(
                        new InstantCommand(() -> {setState(FSMStates.FOLD); extension.setManualControl(false);}),
                        new ConditionalCommand(
                                new IntakeControlCommand(intake, IntakeConstants.backClosedPos, 0),
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                reverseClaw::get),
                        new ConditionalCommand(
                                new IntakeRetractReversedCommand(wrist, pivot, extension),
                                new IntakeRetractCommand(wrist, pivot, extension),
                                reverseClaw::get))
                );
        ;

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> reverseClaw.set(true)).andThen(new ConditionalCommand(new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1), new InstantCommand(), () -> getState() == FSMStates.INTAKE)));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> reverseClaw.set(false)).andThen(new ConditionalCommand(new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, -1), new InstantCommand(), () -> getState() == FSMStates.INTAKE)));

        //driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new IntakeControlCommand(intake, IntakeConstants.backPos, -1))
        //driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeSpinCommand(intake, 0));


        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                // open claw
                                new ConditionalCommand(
                                        new IntakeControlCommand(intake, IntakeConstants.backPos, -1), new IntakeControlCommand(intake, IntakeConstants.openPos, 1), reverseClaw::get),

                                // eject
                                new ConditionalCommand(
                                        new IntakeControlCommand(intake, IntakeConstants.backSinglePos, 0.5), new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, -0.5), reverseClaw::get),

                                () -> getState() == FSMStates.INTAKE)
                )

                .whenReleased(
                        new ConditionalCommand(
                                // intake
                                new ConditionalCommand(
                                        new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1), new IntakeControlCommand(intake, IntakeConstants.closedPos, 1), reverseClaw::get),

                                // not intake
                                new ConditionalCommand(
                                        new IntakeControlCommand(intake, IntakeConstants.backClosedPos, 0), new IntakeControlCommand(intake, IntakeConstants.closedPos, 0), reverseClaw::get
                                ),

                                () -> getState() == FSMStates.INTAKE)
                );

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(bucketPos());
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setState(FSMStates.HANG)),
                        new PivotCommand(pivot, PivotConstants.topLimit)
                ));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ConditionalCommand(
                new PivotCommand(pivot, PivotConstants.hangDegrees), new InstantCommand(), () -> getState() == FSMStates.HANG));

        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(extension::reset));
        operatorPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> extension.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            extension.setManualControl(true);
            if (getState() != FSMStates.INTAKE) {
                setState(FSMStates.NONE);
            }
        }));
        operatorPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> pivot.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            pivot.setManualControl(true);
            setState(FSMStates.NONE);
        }));

        operatorPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                        new ConditionalCommand(new IntakeControlCommand(intake, IntakeConstants.backClosedPos, -1),
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1), reverseClaw::get))
                .whenReleased(new IntakeSpinCommand(intake, 0));

        operatorPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                        new ConditionalCommand(new IntakeClawCommand(intake, IntakeConstants.backPos),
                                new IntakeClawCommand(intake, IntakeConstants.openPos), reverseClaw::get))
                .whenReleased(new ConditionalCommand(new IntakeClawCommand(intake, IntakeConstants.backClosedPos),
                        new IntakeClawCommand(intake, IntakeConstants.closedPos), reverseClaw::get));

        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 15),
                SubPosCommand.newWithExtension(extension, wrist, intake, 7),
                reverseClaw::get
        ));

        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 7),
                SubPosCommand.newWithExtension(extension, wrist, intake, 4),
                reverseClaw::get
        ).alongWith(new InstantCommand(() -> setState(FSMStates.INTAKE))));

        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 15),
                SubPosCommand.newWithExtension(extension, wrist, intake, 10),
                reverseClaw::get
        ).alongWith(new InstantCommand(() -> setState(FSMStates.INTAKE))));

        driverPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(new InstantCommand(() -> setState(FSMStates.SPECIMEN))));
        driverPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(new SpecimenHookCommand(pivot, extension, wrist, intake), new InstantCommand(), () -> getState() == FSMStates.SPECIMEN));

        new Trigger(() -> true).whileActiveContinuous(
                new ConditionalCommand(
                        new InstantCommand(() -> {
                            double power = gamepad2.right_trigger + gamepad1.right_trigger * manualMultiplier - gamepad2.left_trigger - gamepad1.left_trigger * manualMultiplier;
                            if (getState() == FSMStates.INTAKE
                                    && extension.getCurrentInches() > SlideConstants.submersibleIntakeMaxExtension
                                    && power > 0) {
                                power = 0;
                            }
                            extension.setPower(power);
                        }),
                        new InstantCommand(),
                        extension::getManualControl), true);

                /*new RunCommand(() -> extension.setPower(- gamepad2.left_trigger - gamepad1.left_trigger), extension),
                () -> extension.getCurrentPosition() / SlideConstants.ticksPerInch < SlideConstants.submersibleIntakeMaxExtension))*/

        new Trigger(pivot::getManualControl).toggleWhenActive(new RunCommand(() -> pivot.setPower(gamepad2.right_stick_x), pivot));


        new Trigger(() -> gamepad1.options).whileActiveOnce(new InstantCommand(pinpoint::resetYaw)); // pinpoint.resetYaw()));
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentInches());
            telemetry.addData("pivotpos", pivot.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            telemetry.addData("intake flipped?", reverseClaw.get());
            timer.reset();
            update();
        }
        CommandScheduler.getInstance().reset();
    }
}
