package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SubPosCommand extends SequentialCommandGroup {
    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(new ExtendCommand(extension, 4), new WristCommand(wrist, IntakeConstants.groundPos));
    }
}