package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new ExtendCommand(extension, 1),
                new PivotCommand(pivot, 90),
                new ExtendCommand(extension, SlideConstants.maxExtension).withTimeout(1000),
                new WristCommand(wrist, IntakeConstants.scoringPos));
    }
}
