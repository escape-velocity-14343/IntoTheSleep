package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenHookCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SpecimenRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.opmode.auton.PU5Apple;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoSpecimenScoreCommand extends SequentialCommandGroup {

    public AutoSpecimenScoreCommand(PivotSubsystem pivot, ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, DefaultGoToPointCommand gtpc, PinpointSubsystem pinpoint, MecanumDriveSubsystem mecanum, double scorePos) {
        addCommands(
        new GoToPointWithDefaultCommand(new Pose2d(-50, -40, new Rotation2d()), gtpc).alongWith(
                new SpecimenRaiseCommand(pivot, extension, wrist).withTimeout(3000),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
        ),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 1),
                new GoToPointWithDefaultCommand(PU5Apple.intakePos, gtpc).interruptOn(() -> pinpoint.getVelocity().getTranslation().getNorm() < PU5Apple.intakeStallVelocity),
                // move forward slightly while intaking
                new InstantCommand(() -> gtpc.setToggle(false)),
                new WaitCommand(1000).alongWith(new IntakeClawCommand(intake, IntakeConstants.closedPos), new RunCommand(() -> mecanum.driveFieldCentric(-0.3, 0, 0)).withTimeout(1000)),
                new InstantCommand(() -> gtpc.setToggle(true)),
                new SpecimenHookCommand(pivot, extension, wrist, intake).alongWith(
                        new GoToPointWithDefaultCommand(new Pose2d(-50, scorePos, new Rotation2d()), gtpc, 5, 10).interruptOn(() -> pinpoint.getPose().getY() > -12)).withTimeout(1500),
                new GoToPointWithDefaultCommand(new Pose2d(-30, scorePos, new Rotation2d()), gtpc).interruptOn(() -> pinpoint.getPose().getX() > -32).withTimeout(1500),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                new WaitCommand(50)
        );
    }

}
