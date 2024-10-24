package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.IntakeConstants;
import org.firstinspires.ftc.teamcode.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new ExtendCommand(extension, 0),
                new PivotCommand(pivot, PivotConstants.topLimit),
                new ExtendCommand(extension, 26),
                new WristCommand(wrist, 0.5));
    }
}
