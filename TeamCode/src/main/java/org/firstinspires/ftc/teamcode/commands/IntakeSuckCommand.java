package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakeSuckCommand extends SequentialCommandGroup {
    public IntakeSuckCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(new ExtendCommand(extension, 4), new WristCommand(wrist, IntakeConstants.openPos), new IntakeSpinCommand(intake, 1));
    }
}
