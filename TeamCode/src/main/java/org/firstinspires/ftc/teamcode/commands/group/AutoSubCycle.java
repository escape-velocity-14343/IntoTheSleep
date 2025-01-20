package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import android.util.Log;

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

public class AutoSubCycle extends SequentialCommandGroup {

    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub) {
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),
                        new SequentialCommandGroup(
                        new GoToPointWithDefaultCommand(new Pose2d(-15, 47, Rotation2d.fromDegrees(-90)), gtpc, 5, 20)
                                .interruptOn(() -> pinpoint.getPose().getX() > -17),
                        new ConditionalCommand(
                                new GoToPointWithDefaultCommand(new Pose2d(-15, 22, Rotation2d.fromDegrees(-90)), gtpc, 2, 4).withTimeout(500),
                                new GoToPointWithDefaultCommand(new Pose2d(-9, 24, Rotation2d.fromDegrees(-90)), gtpc, 2, 4),
                                        () -> clearSub)
                        ).alongWith(new SequentialCommandGroup(new IntakeClawCommand(intake, IntakeConstants.foldedPos), new RetractCommand(wrist, pivot, extension),
                                new SubPosReadyCommand(extension, pivot, wrist, intake, 5))),

                new ConditionalCommand(
                        new WaitCommand(300).alongWith(new SubClearCommand(subClear).withTimeout(10),
                                new GoToPointWithDefaultCommand(new Pose2d(-15, 19, Rotation2d.fromDegrees(-90)), gtpc, 100, 1000).withTimeout(10)),
                        new InstantCommand(),
                        () -> clearSub),

                new GoToPointWithDefaultCommand(new Pose2d(-9, 24, Rotation2d.fromDegrees(-90)), gtpc, 3, 4),
                new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                /*new ConditionalCommand(
                        new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),
                        new InstantCommand(),
                        () -> !(cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED))
                ),*/

                new GoToPointWithDefaultCommand(new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getX() < -25),
                new BucketPosCommand(extension, pivot, wrist).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)),

                new IntakeControlCommand(intake, IntakeConstants.openPos, -1),
                new WaitCommand(350));
}

    public AutoSubCycle(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, CameraSubsystem cam, SubClearSubsystem subClear, PinpointSubsystem pinpoint, DefaultGoToPointCommand gtpc, boolean clearSub, Pose2d subIntakePos){
        addCommands
                (
                        new InstantCommand(() -> cam.setEnabled(true)),
                        new SequentialCommandGroup(
                                new GoToPointWithDefaultCommand(new Pose2d(-6, 40, Rotation2d.fromDegrees(-90)), gtpc, 20, 20)
                                        .interruptOn(() -> pinpoint.getPose().getX() > -20),
                                new ConditionalCommand(
                                        new GoToPointWithDefaultCommand(subIntakePos.transformBy(new Transform2d(new Translation2d(3, -3), new Rotation2d())), gtpc, 4, 4).withTimeout(500),
                                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 1, 4),
                                        () -> clearSub)
                        ).alongWith(new SequentialCommandGroup(new IntakeClawCommand(intake, IntakeConstants.foldedPos), new RetractCommand(wrist, pivot, extension))),

                        new ConditionalCommand(
                                new WaitCommand(300).alongWith(new SubClearCommand(subClear).andThen(new SubClearCommand(subClear))), new InstantCommand(),
                                () -> clearSub),

                        new GoToPointWithDefaultCommand(subIntakePos, gtpc, 4, 4),

                        new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),

                        new ConditionalCommand(
                                new AutoSubIntake(extension, wrist, cam, gtpc, intake, pinpoint, pivot),
                                new InstantCommand(),
                                () -> cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)
                        ),


                        new GoToPointWithDefaultCommand(new Pose2d(-48, 48, Rotation2d.fromDegrees(-45)), gtpc, 8, 30).interruptOn(()->pinpoint.getPose().getX() < -44),
                        new BucketPosCommand(extension, pivot, wrist).alongWith(
                                new GoToPointWithDefaultCommand(scorePos, gtpc)),

                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, -1),
                        new WaitCommand(500));
    }

}
