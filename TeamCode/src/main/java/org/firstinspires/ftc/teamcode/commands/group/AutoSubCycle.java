package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.autoscoreMaxVel;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.outtakePause;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.outtakeTimeout;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.subBarrierY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SubClearCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SubClearWipeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WaitUntilStabilizedCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

@Config
public class AutoSubCycle extends SequentialCommandGroup {

    public static double subBarrierCycleOffset = 2;

    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub) {
        addCommands
                (
                new InstantCommand(() -> cam.setEnabled(true)),

                new SequentialCommandGroup(
                        new GoToPointWithDefaultCommand(new Pose2d(-12, 37, Rotation2d.fromDegrees(-60)), gtpc, 5, 20)
                                .interruptOn(() -> pinpoint.getPose().getX() > -16),
                        new ConditionalCommand(
                                new GoToPointWithDefaultCommand(new Pose2d(-12, 15, Rotation2d.fromDegrees(-90)), gtpc, 2, 4).withTimeout(1250),
                                new GoToPointWithDefaultCommand(() -> new Pose2d(-15, subBarrierY + subBarrierCycleOffset, Rotation2d.fromDegrees(-90)), gtpc, 2, 4).interruptOn(() -> pinpoint.getPose().getY() < subBarrierY + subBarrierCycleOffset),
                                        () -> clearSub)
                ).alongWith(
                        new SequentialCommandGroup(
                                new IntakeClawCommand(intake, IntakeConstants.foldedPos),
                                new RetractCommand(wrist, pivot, extension),
                                new SubPosReadyCommand(extension, pivot, wrist, intake, 4)
                        )
                ),

                new ConditionalCommand(
                        // if first time, set sub barrier y
                        new InstantCommand(() -> {
                            if (Math.abs(subBarrierY - pinpoint.getPose().getY()) < 7.5) {
                                AutoConstants.subBarrierY = pinpoint.getPose().getY();
                            }
                            Log.v("subBarrierY", "new: " + pinpoint.getPose().getY());
                        }).alongWith(new SubClearWipeCommand(subClear)),
                        new InstantCommand(),
                        () -> clearSub),

                new GoToPointWithDefaultCommand(() -> new Pose2d(pinpoint.getPose().getX(), subBarrierY + subBarrierCycleOffset, Rotation2d.fromDegrees(-90)), gtpc)
                        .interruptOn(() -> subBarrierY + subBarrierCycleOffset - 1.5 < pinpoint.getPose().getY() && pinpoint.getPose().getY() < subBarrierY + subBarrierCycleOffset),
                new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                /*new ConditionalCommand(
                        new GoToPointWithDefaultCommand(new Pose2d(-15, 29, Rotation2d.fromDegrees(-90)), gtpc, 3, 4)
                                .andThen(
                                    new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot)
                                ),
                        new InstantCommand(),
                        () -> !(cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED))
                ),*/

                new GoToPointWithDefaultCommand(new Pose2d(-20, 45, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getY() > 35).alongWith(
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                        new IntakeRetractCommand(wrist, pivot, extension)
                ),
                        new GoToPointWithDefaultCommand(scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist)
                        ).withTimeout(outtakeTimeout),

                        new WaitUntilStabilizedCommand(pinpoint),

                new IntakeControlCommand(intake, IntakeConstants.openPos, -1), new WaitCommand(outtakePause)
        );
}

    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, Pose2d subIntakePos){
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),

                        new SequentialCommandGroup(
                                new GoToPointWithDefaultCommand(new Pose2d(-12, 37, Rotation2d.fromDegrees(-60)), gtpc, 5, 20)
                                        .interruptOn(() -> pinpoint.getPose().getX() > -16),
                                new GoToPointWithDefaultCommand(subIntakePos, gtpc, 2, 4).interruptOn(() -> pinpoint.getPose().getY() < subIntakePos.getY())
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new IntakeClawCommand(intake, IntakeConstants.foldedPos),
                                        new RetractCommand(wrist, pivot, extension),
                                        new SubPosReadyCommand(extension, pivot, wrist, intake, 4)
                                )
                        ),

                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 3, 4).interruptOn(() -> subIntakePos.getY() - 1.5 < pinpoint.getPose().getY() && pinpoint.getPose().getY() < subIntakePos.getY()),
                        new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                        new GoToPointWithDefaultCommand(new Pose2d(-20, 45, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getY() > 35).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new IntakeRetractCommand(wrist, pivot, extension)
                        ),
                        new BucketPosCommand(extension, pivot, wrist).alongWith(
                                new GoToPointWithDefaultCommand(scorePos, gtpc)),

                        new WaitUntilStabilizedCommand(pinpoint),

                        new IntakeControlCommand(intake, IntakeConstants.openPos, -1), new WaitCommand(outtakePause)
                );
    }

    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, Pose2d subIntakePos, Pose2d subClearPos){
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),

                        new SequentialCommandGroup(
                                new GoToPointWithDefaultCommand(new Pose2d(-12, 37, Rotation2d.fromDegrees(-60)), gtpc, 5, 20)
                                        .interruptOn(() -> pinpoint.getPose().getX() > -16),
                                new GoToPointWithDefaultCommand(subClearPos, gtpc, 2, 4).withTimeout(750)
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new IntakeClawCommand(intake, IntakeConstants.foldedPos),
                                        new RetractCommand(wrist, pivot, extension),
                                        new SubPosReadyCommand(extension, pivot, wrist, intake, 4)
                                )
                        ),

                        new SubClearWipeCommand(subClear),

                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 3, 4).interruptOn(() -> subIntakePos.getY() - 1.5 < pinpoint.getPose().getY() && pinpoint.getPose().getY() < subIntakePos.getY()),
                        new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                /*new ConditionalCommand(
                        new GoToPointWithDefaultCommand(new Pose2d(-15, 29, Rotation2d.fromDegrees(-90)), gtpc, 3, 4)
                                .andThen(
                                    new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot)
                                ),
                        new InstantCommand(),
                        () -> !(cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED))
                ),*/

                        new GoToPointWithDefaultCommand(new Pose2d(-20, 45, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getY() > 35).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new IntakeRetractCommand(wrist, pivot, extension)
                        ),
                        new BucketPosCommand(extension, pivot, wrist).alongWith(
                                new GoToPointWithDefaultCommand(scorePos, gtpc)),

                        new WaitUntilStabilizedCommand(pinpoint),

                        new IntakeControlCommand(intake, IntakeConstants.openPos, -1), new WaitCommand(outtakePause)
                );
    }

    @Deprecated
    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub, Pose2d subIntakePos){
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),

                        new SequentialCommandGroup(
                                new GoToPointWithDefaultCommand(new Pose2d(-12, 37, Rotation2d.fromDegrees(-60)), gtpc, 5, 20)
                                        .interruptOn(() -> pinpoint.getPose().getX() > -20),
                                new ConditionalCommand(
                                        new GoToPointWithDefaultCommand(new Pose2d(subIntakePos.getX(), subIntakePos.getY() - 14, subIntakePos.getRotation()), gtpc, 2, 4).withTimeout(750),
                                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 2, 4).interruptOn(() -> pinpoint.getPose().getY() < subIntakePos.getY()),
                                        () -> clearSub)
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new IntakeClawCommand(intake, IntakeConstants.foldedPos),
                                        new RetractCommand(wrist, pivot, extension),
                                        new SubPosReadyCommand(extension, pivot, wrist, intake, 6)
                                )
                        ).deadlineWith(
                                new ConditionalCommand(
                                        new SubClearCommand(subClear).withTimeout(5000),
                                        new InstantCommand(),
                                        () -> clearSub)),

                        new GoToPointWithDefaultCommand(new Pose2d(-12, 29, Rotation2d.fromDegrees(-60)), gtpc, 3, 4).interruptOn(() -> 27.5 < pinpoint.getPose().getY() && pinpoint.getPose().getY() < 29),
                        new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                /*new ConditionalCommand(
                        new GoToPointWithDefaultCommand(new Pose2d(-15, 29, Rotation2d.fromDegrees(-90)), gtpc, 3, 4)
                                .andThen(
                                    new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot)
                                ),
                        new InstantCommand(),
                        () -> !(cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED))
                ),*/

                        new GoToPointWithDefaultCommand(new Pose2d(-20, 45, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getY() > 35).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new IntakeRetractCommand(wrist, pivot, extension)
                        ),
                        new BucketPosCommand(extension, pivot, wrist).alongWith(
                                new GoToPointWithDefaultCommand(scorePos, gtpc)),

                        new IntakeControlCommand(intake, IntakeConstants.openPos, -1), new WaitCommand(outtakePause)
                );
    }

}
