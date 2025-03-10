package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.autoscoreMaxVel;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.outtakePause;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.ProxyScheduleCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.BasketAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.DrivebasePowerCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SubClearCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WaitUntilStabilizedCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractReversedCommand;
import org.firstinspires.ftc.teamcode.commands.group.L3HangCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReversedCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;

import java.util.concurrent.atomic.AtomicBoolean;

import javax.crypto.spec.OAEPParameterSpec;

@TeleOp(group = "0", name = "TeleOppV2")
@Config
public class TeleOppsV2 extends Robot {
    public static double manualMultiplier = 0.7;
    public static double robotMovementMultiplier = 0.5;

    protected GamepadEx driverPad;
    protected GamepadEx operatorPad;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driverPad = new GamepadEx(gamepad1);
        operatorPad = new GamepadEx(gamepad2);

        if (true) {
            CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                    mecanum,
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> pinpoint.getPose().getRotation().getDegrees()
            ) {
                //@Override
                //public double getXModPower() {
                //    if (getState() != FSMStates.SPECIMEN) {
                //        return 0.0;
                //    }
                //
                //    return
                //}
            });
        } else {
            DefaultDriveCommand drive = new DefaultDriveCommand(mecanum,
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftY() / driverPad.getLeftX() < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftX() / driverPad.getLeftY() < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> 0.0);
            CommandScheduler.getInstance().setDefaultCommand(mecanum, drive);
        }
        //driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist, pivot, extension));


        configureDriver();
        configureOperator();
        configureDualControl();

        /*new RunCommand(() -> extension.setPower(- gamepad2.left_trigger - gamepad1.left_trigger), extension),
        () -> extension.getCurrentPosition() / SlideConstants.ticksPerInch < SlideConstants.submersibleIntakeMaxExtension))*/

        // pinpoint.resetYaw()));
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentInches());
            telemetry.addData("pivotpos", pivot.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            telemetry.addData("intake flipped?", reverseClaw.get());
            telemetry.addData("intake front voltage", intake.getFrontV());
            telemetry.addData("intake back voltage", intake.getBackV());
            telemetry.addData("pose x", pinpoint.getPose().getX());
            telemetry.addData("pose y", pinpoint.getPose().getY());
            telemetry.addData("pose heading", pinpoint.getPose().getRotation().getDegrees());
            //if (intake.getFrontV()>IntakeConstants.intakeSensorVoltageThres) {
            //    gamepad1.rumble(100);
            //}
            timer.reset();
            update();
        }
        CommandScheduler.getInstance().reset();
        pto.setPto(AscentSubsytem.PTOMode.STOWED);
    }

    public void configureDriver() {
        // ------- UTILITIES -------
        // heading reset
        new Trigger(() -> gamepad1.options).whileActiveOnce(new InstantCommand(pinpoint::resetYaw));


        // ------- SPECIMEN --------
        // specimen scoring
        driverPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SpecimenRaiseCommand(pivot, extension, wrist).alongWith(new InstantCommand(() -> setState(FSMStates.SPECIMEN))));
        driverPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(new SpecimenHookCommand(pivot, extension, wrist, intake), new InstantCommand(), () -> getState() == FSMStates.SPECIMEN));


        // -------- BUCKET --------
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(bucketPos());

        // true bucket autoscore
        ElapsedTime bucketTimer = new ElapsedTime();
        AtomicBoolean doneAutoscoring = new AtomicBoolean(false);
        new Trigger(() -> getState() == FSMStates.AUTOSCORE || bucketTimer.seconds() < 0.25 || gamepad1.touchpad)
                .whileActiveOnce(
                        new InstantCommand(() -> setState(FSMStates.AUTOSCORE)).andThen(

                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new WaitCommand(100),

                                // go to bucket
                                new ParallelCommandGroup(

                                        new InstantCommand(bucketTimer::reset),
                                        new InstantCommand(() -> doneAutoscoring.set(false)),
                                        new InstantCommand(() -> setState(FSMStates.AUTOSCORE)),

                                        // retract and then go to bucket pos and score
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(

                                                        // retract and then go to bucket pos
                                                        new InterruptCommand(new RetractCommand(wrist, pivot, extension),
                                                                () -> Util.pose2dToDistance(scorePos, pinpoint.getPose()) < 48.0
                                                        ),
                                                        new WaitUntilCommand(
                                                                () -> Util.pose2dToDistance(scorePos, pinpoint.getPose()) < 48.0
                                                        ),
                                                        bucketPos(),

                                                        // wait until we get to position
                                                        new WaitUntilCommand(
                                                                () -> Util.pose2dToDistance(scorePos, pinpoint.getPose()) < 2.0
                                                        ),

                                                        // wait until we get stable, then score
                                                        new WaitUntilStabilizedCommand(pinpoint),
                                                        new IntakeControlCommand(intake, IntakeConstants.openPos, 0),
                                                        new WaitCommand(outtakePause),
                                                        new InstantCommand(() -> Log.i("%Autoscore", "Autoscore done")),
                                                        new InstantCommand(() -> doneAutoscoring.set(true))

                                                ),
                                                // while we score. align with bucket
                                                new InterruptCommand(new BasketAlignCommand(mecanum, basketSensor, pinpoint, 2)
                                                        .withXySupplier(() -> gamepad1.touchpad_finger_1_y * 3, () -> gamepad1.touchpad_finger_1_x * 4)
                                                        // allow movement interrupt without breaking everything
                                                        .setInterrupt(() -> bucketTimer.seconds() > 0.25
                                                                && (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.05
                                                                || Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) > 0.05)
                                                        ), doneAutoscoring::get)
                                        )

                                        // if we score prematurely, also interrupt
                                ).interruptOn(() -> bucketTimer.seconds() > 0.75 && gamepad1.left_bumper),

                                new InstantCommand(() -> gamepad1.rumble(30, 30, 50)),

                                new ParallelCommandGroup(
                                        new InterruptCommand(
                                                new TimeoutCommand(new RunCommand(() -> mecanum.driveFieldCentric(-3, 1, 0), mecanum), 1000),
                                                () -> (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.05
                                                        || Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) > 0.05)
                                        ),
                                        new SequentialCommandGroup(
                                                new WristCommand(wrist, IntakeConstants.halfFoldPos),
                                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                                                new InterruptCommand(
                                                        new RetractCommand(wrist, pivot, extension),
                                                        () -> extension.getCurrentInches() < 0
                                                ),
                                                new SubPosReadyCommand(extension, pivot, wrist, intake, SlideConstants.submersibleIntakeMaxExtension - 2)
                                        )
                                )

                        )
                );

        // toggle high extend mode
        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> DriveConstants.highExtend = true));
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> DriveConstants.highExtend = false));

        // --------- RETRACT --------
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist, pivot, extension));

        // --------- INTAKE --------
        // drop intake
        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(

                        new ConditionalCommand(
                                new WaitUntilCommand(() -> !gamepad2.options).andThen(
                                        new WaitCommand((long) IntakeConstants.subClearMillis)),
                                new InstantCommand(),
                                () -> gamepad2.options
                        ),
                        new ConditionalCommand(
                                new RetractCommand(wrist, pivot, extension),
                                new InstantCommand(),
                                () -> pivot.getCurrentPosition() > 5),
                        subPos()
                ))

                .whenReleased(
                        // only retract if not autoscoring
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0).andThen(
                                new RunIfCommand(new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            setState(FSMStates.FOLD);
                                            extension.setManualControl(false);
                                        }),
                                        new IntakeRetractCommand(wrist, pivot, extension)), () -> getState() != FSMStates.AUTOSCORE)
                        ));

        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                // intake
                                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                                // not intake
                                new IntakeControlCommand(intake, IntakeConstants.openPos, 0),
                                () -> getState() == FSMStates.INTAKE || getState() == FSMStates.SPECIMEN)
                )
                .whenReleased(
                        new ConditionalCommand(
                                // intake
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                                // not intake
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                () -> getState() == FSMStates.INTAKE || getState() == FSMStates.SPECIMEN)
                );

        // -------- ASCENT ---------
        // ascent up, ascent down
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setState(FSMStates.HANG)),
                        new PivotCommand(pivot, PivotConstants.topLimit)
                ));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ConditionalCommand(
                new ParallelCommandGroup(
                        new DrivebasePowerCommand(mecanum, 0.0, 1.0, 0.0).withTimeout(1000),
                        new WaitCommand(250).andThen(new PivotCommand(pivot, PivotConstants.hangDegrees))
                ),
                new InstantCommand(),
                () -> getState() == FSMStates.HANG
        ));
        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ConditionalCommand(
                new L3HangCommand(pto, pivot, wrist, extension, mecanum, () -> gamepad1.dpad_left),
                new InstantCommand(),
                () -> getState() == FSMStates.HANG
        ));
        new Trigger(() -> intake.getFrontV() > IntakeConstants.intakeSensorVoltageThres && getState() == FSMStates.INTAKE)
                .whileActiveContinuous(new InstantCommand(() -> gamepad1.rumble(0.5, 0.5, 100)));
    }

    public void configureOperator() {
        // --------- UTILITIES ------------
        // slide reset
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(extension::reset));
        // sub clear
        new Trigger(() -> gamepad2.options).whileActiveOnce(new SubClearCommand(subClear)
                .interruptOn(() -> !gamepad2.options));
                /*.alongWith(
                        new InstantCommand(() -> )
                ));*/

        // ---------- PIVOT -----------
        // pivot manual control
        new Trigger(pivot::getManualControl).toggleWhenActive(new RunCommand(() -> pivot.setPower(gamepad2.right_stick_x), pivot));

        // ---------- MANUAL CONTROL TOGGLES -----------
        // extension manual toggles
        operatorPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> extension.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            extension.setManualControl(true);
            if (getState() != FSMStates.INTAKE) {
                setState(FSMStates.NONE);
            }
        }));
        // pivot manual toggles
        operatorPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> pivot.setManualControl(false)));
        operatorPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            pivot.setManualControl(true);
            setState(FSMStates.NONE);
        }));

        // ---------- INTAKE ----------
        // run intake
        operatorPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                        new ConditionalCommand(new IntakeControlCommand(intake, IntakeConstants.backClosedPos, -1),
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1), reverseClaw::get))
                .whenReleased(new IntakeSpinCommand(intake, 0));
        // reverse intake in all states
        operatorPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                        new ConditionalCommand(
                                new IntakeControlCommand(intake, IntakeConstants.backSinglePos, 1),
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, -1), reverseClaw::get))
                .whenReleased(new ConditionalCommand(
                        new IntakeControlCommand(intake, IntakeConstants.backClosedPos, 0),
                        new ConditionalCommand(
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                () -> getState() == FSMStates.INTAKE
                        ),
                        reverseClaw::get));

        // -------- INTAKE PRESETS ---------

        // short
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 7),
                new SubPosReadyCommand(extension, pivot, wrist, intake, 4),
                reverseClaw::get
        ));

        // medium
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 12),
                new SubPosReadyCommand(extension, pivot, wrist, intake, 10),
                reverseClaw::get));

        // long
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ConditionalCommand(
                SubPosReversedCommand.newWithExtension(extension, wrist, intake, pivot, 15),
                new SubPosReadyCommand(extension, pivot, wrist, intake, 15),
                reverseClaw::get
        ));

        // ------- BUCKET --------
        // low bucket toggle
        operatorPad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> lowBucket.set(true)));
        operatorPad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> lowBucket.set(false)));


    }

    public void configureDualControl() {
        // slide manual control
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


    }
}

//  ______________
// ||            ||
// ||            ||
// ||____________||
// |______________|
// \\#####EV#####\\
//  \\############\\
//   \      ____    \
//    \_____\___\____\