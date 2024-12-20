package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.scorePos;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.AutonExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class Auton3Yellows extends SequentialCommandGroup {
    public Auton3Yellows(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, DefaultGoToPointCommand gtpc) {
        addRequirements(extension, pivot, wrist, intake);
        addCommands(
                // first sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 48, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos).andThen(new IntakePosCommand(extension, pivot, wrist))
                ).withTimeout(3000),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension).interruptOn(intake::getDSensorSupplier),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                //RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos)
                new BucketPosCommand(extension, pivot, wrist).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                //new WaitCommand(100),

                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                //new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                //new IntakeClawCommand(intake, IntakeConstants.closedPos),

                // second sample
                new GoToPointWithDefaultCommand(new Pose2d(-44, 58, Rotation2d.fromDegrees(0)), gtpc).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos).andThen(new IntakePosCommand(extension, pivot, wrist))
                ).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece1Extension).interruptOn(intake::getDSensorSupplier),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                //RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos)
                new BucketPosCommand(extension, pivot, wrist).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                //new WaitCommand(100),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                //new WristCommand(wrist, IntakeConstants.bucketRetractPos),
                //new IntakeClawCommand(intake, IntakeConstants.closedPos),

                new GoToPointWithDefaultCommand(new Pose2d(-44, 58, Rotation2d.fromDegrees(29)), gtpc).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos).andThen(new IntakePosCommand(extension, pivot, wrist))
                ).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos - 0.02, 1),
                new AutonExtendCommand(extension, SlideConstants.autonPiece3Extension).withTimeout(1500).interruptOn(intake::getDSensorSupplier),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                //RetractCommand.newWithWristPos(wrist, pivot, extension, IntakeConstants.groundPos)
                new BucketPosCommand(extension, pivot, wrist).alongWith(
                        new GoToPointWithDefaultCommand(scorePos, gtpc)
                ),
                //new WaitCommand(100),
                new IntakeControlCommand(intake,IntakeConstants.singleIntakePos, -1),
                new WaitCommand(500),
                new IntakeSpinCommand(intake, 0),
                new WristCommand(wrist, IntakeConstants.groundPos)
                //new IntakeClawCommand(intake, IntakeConstants.closedPos)

        );
    }
}
