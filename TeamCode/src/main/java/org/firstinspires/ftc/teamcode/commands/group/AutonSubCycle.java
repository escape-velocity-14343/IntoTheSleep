package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

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

public class AutonSubCycle extends SequentialCommandGroup {

    public AutonSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub) {
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),
                        new SequentialCommandGroup(
                        new GoToPointWithDefaultCommand(new Pose2d(-6, 40, Rotation2d.fromDegrees(-90)), gtpc, 20, 20)
                                .interruptOn(() -> pinpoint.getPose().getX() > -20),
                        new ConditionalCommand(
                                new GoToPointWithDefaultCommand(new Pose2d(-13, 22, Rotation2d.fromDegrees(-90)), gtpc, 1, 4).withTimeout(500),
                                new GoToPointWithDefaultCommand(new Pose2d(-10, 25, Rotation2d.fromDegrees(-90)), gtpc, 1, 4),
                                        () -> clearSub)
                        ).alongWith(new SequentialCommandGroup(new IntakeClawCommand(intake, IntakeConstants.foldedPos), new RetractCommand(wrist, pivot, extension))),

                new ConditionalCommand(
                        new WaitCommand(300).alongWith(new SubClearCommand(subClear)), new InstantCommand(),
                        () -> clearSub),

                new GoToPointWithDefaultCommand(new Pose2d(-10, 25, Rotation2d.fromDegrees(-90)), gtpc, 1, 4),
                new ExtendCommand(extension, 4),
                new WristCommand(wrist, IntakeConstants.groundPos).alongWith(
                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos-0.025, 1)),
                new WaitCommand(250),
                //new SampleAutoAlign(cam, gtpc, pinpoint).deadlineWith(
                //        new AutonExtendCommand(extension, SlideConstants.submersibleIntakeMaxExtension)).withTimeout(2000),
                new SampleAutoAlignAndExtend(cam, gtpc, pinpoint, extension).withTimeout(2000),

                new ConditionalCommand(
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0.5),
                        new IntakeControlCommand(intake, IntakeConstants.openPos, -0.5),
                        () -> cam.getYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)),

                new SequentialCommandGroup
                        (new IntakeRetractCommand(wrist, pivot, extension),
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new PivotCommand(pivot, PivotConstants.topLimit-1),
                                new WaitUntilCommand(() -> Util.pose2dToDistance(pinpoint.getPose(), scorePos) < 48),
                                new BucketPosCommand(extension, pivot, wrist))
                        .alongWith(new GoToPointWithDefaultCommand(scorePos, gtpc)),

                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500));
    }

    public AutonSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub, Pose2d subIntakePos){
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),
                        new SequentialCommandGroup(
                                new GoToPointWithDefaultCommand(new Pose2d(-6, 40, Rotation2d.fromDegrees(-90)), gtpc, 20, 20)
                                        .interruptOn(() -> pinpoint.getPose().getX() > -20),
                                new ConditionalCommand(
                                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 1, 4).withTimeout(500),
                                        new GoToPointWithDefaultCommand(new Pose2d(subIntakePos.getX()+3, subIntakePos.getY()+3, subIntakePos.getRotation()), gtpc, 1, 4),
                                        () -> clearSub)
                        ).alongWith(new SequentialCommandGroup(new IntakeClawCommand(intake, IntakeConstants.foldedPos), new RetractCommand(wrist, pivot, extension))),

                        new ConditionalCommand(
                                new WaitCommand(300).alongWith(new SubClearCommand(subClear)), new InstantCommand(),
                                () -> clearSub),

                        new GoToPointWithDefaultCommand(new Pose2d(subIntakePos.getX()+3, subIntakePos.getY()+3, subIntakePos.getRotation()), gtpc, 1, 4),
                        new ExtendCommand(extension, 4),
                        new WristCommand(wrist, IntakeConstants.groundPos).alongWith(
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos-0.025, 1)),
                        new WaitCommand(250),
                        new SampleAutoAlign(cam, gtpc, pinpoint).deadlineWith(
                                new AutonExtendCommand(extension, SlideConstants.submersibleIntakeMaxExtension)).withTimeout(2000),

                        new ConditionalCommand(
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0.5),
                                new IntakeControlCommand(intake, IntakeConstants.openPos, -0.5),
                                () -> cam.getYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)),

                        new SequentialCommandGroup
                                (new IntakeRetractCommand(wrist, pivot, extension),
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                        new PivotCommand(pivot, PivotConstants.topLimit-1),
                                        new WaitUntilCommand(() -> Util.pose2dToDistance(pinpoint.getPose(), scorePos) < 48),
                                        new BucketPosCommand(extension, pivot, wrist))
                                .alongWith(new GoToPointWithDefaultCommand(scorePos, gtpc)),

                        new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                        new WaitCommand(500));
    }

}
